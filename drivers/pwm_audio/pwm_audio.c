/*
 * frank-msx — fMSX for RP2350
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://github.com/rh1tech/frank-msx
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * frank-msx - PWM Audio Driver
 *
 * Architecture:
 *   - One PWM slice drives the speaker pin(s). Its carrier runs at
 *     PWM_CARRIER_MULT x the sample rate (~110 kHz at 22.05 kHz), well
 *     above the audio band, with ~11 bits of amplitude resolution at
 *     sys_clk = 252 MHz.
 *   - A sample ring (RING_LEN packed CC words, low16 = CC_A, high16 =
 *     CC_B) is read by one endless DMA channel in ring mode, paced by a
 *     DMA timer at the sample rate, into the slice's CC register. New
 *     CC values latch at the next PWM wrap, so there is no tearing.
 *   - The emulator pushes samples in small pieces (fMSX renders sound
 *     every 8 scanlines, ~11 samples at a time). They are appended to the
 *     ring right behind the DMA read pointer, so the stream stays
 *     continuous: no padding with silence, no dropped blocks.
 *   - The fill level is held near RING_TARGET by inserting or skipping a
 *     single sample per push, which absorbs the small rate difference
 *     between the emulated machine and the DMA timer.
 *   - Boards whose two audio pins sit on different PWM slices (Olimex
 *     PICO-PC: GPIO27 = slice 5 B, GPIO28 = slice 6 A) get a second DMA
 *     channel + timer that copies the same ring into the other slice.
 *
 * SPDX-License-Identifier: MIT
 */

#include "pwm_audio.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include <stdio.h>
#include <string.h>

/* PWM carrier relative to the sample rate. */
#define PWM_CARRIER_MULT 5

/* Sample ring: 2048 samples = ~93 ms at 22.05 kHz; a power of two, aligned
 * to its size in bytes for DMA ring addressing. */
#define RING_BITS    11
#define RING_LEN     (1u << RING_BITS)
#define RING_MASK    (RING_LEN - 1u)
#define RING_BYTES   (RING_LEN * 4u)
#define RING_TARGET  768u      /* ~35 ms of buffered audio */
#define RING_SLACK   256u      /* correct the rate outside target +- slack */
#define RING_MAX     (RING_LEN - 128u)

static uint32_t __attribute__((aligned(RING_BYTES))) ring[RING_LEN];

static uint32_t pwm_wrap = 2047;
static uint32_t pwm_center = 1023;
static uint32_t wr_pos = 0;          /* next ring slot the producer writes */
static uint32_t last_word = 0;

static bool initialized = false;
static int dma_ch = -1, dma_ch_m = -1;
static int dma_timer = -1, dma_timer_m = -1;

static inline uint32_t rd_pos(void) {
    return (((uint32_t)dma_hw->ch[dma_ch].read_addr - (uint32_t)(uintptr_t)ring) / 4u) & RING_MASK;
}

static void setup_channel(int ch, int timer, uint slice) {
    dma_channel_config c = dma_channel_get_default_config((uint)ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, RING_BITS + 2);   /* wrap read addr */
    channel_config_set_dreq(&c, dma_get_timer_dreq((uint)timer));
    dma_channel_configure((uint)ch, &c, &pwm_hw->slice[slice].cc, ring,
                          dma_encode_endless_transfer_count(), false);
}

void pwm_audio_init(uint pin_l, uint pin_r, uint32_t sample_rate) {
    if (initialized) return;

    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t wrap_plus_one = sys_clk / (sample_rate * PWM_CARRIER_MULT);
    if (wrap_plus_one < 2) wrap_plus_one = 2;
    if (wrap_plus_one > 65536) wrap_plus_one = 65536;
    pwm_wrap = wrap_plus_one - 1;
    pwm_center = pwm_wrap / 2;
    uint32_t sample_period = sys_clk / sample_rate;   /* timer = 1/period */
    if (sample_period > 65535) sample_period = 65535;

    last_word = (pwm_center << 16) | pwm_center;
    for (uint32_t i = 0; i < RING_LEN; i++) ring[i] = last_word;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, (uint16_t)pwm_wrap);

    uint slice_l = pwm_gpio_to_slice_num(pin_l);
    uint slice_r = pwm_gpio_to_slice_num(pin_r);
    gpio_set_function(pin_l, GPIO_FUNC_PWM);
    gpio_set_function(pin_r, GPIO_FUNC_PWM);
    pwm_init(slice_l, &cfg, false);
    pwm_set_both_levels(slice_l, (uint16_t)pwm_center, (uint16_t)pwm_center);
    if (slice_r != slice_l) {
        pwm_init(slice_r, &cfg, false);
        pwm_set_both_levels(slice_r, (uint16_t)pwm_center, (uint16_t)pwm_center);
    }

    dma_ch = dma_claim_unused_channel(true);
    dma_timer = dma_claim_unused_timer(true);
    dma_timer_set_fraction((uint)dma_timer, 1, (uint16_t)sample_period);
    setup_channel(dma_ch, dma_timer, slice_l);
    uint32_t start_mask = 1u << dma_ch;
    if (slice_r != slice_l) {
        dma_ch_m = dma_claim_unused_channel(true);
        dma_timer_m = dma_claim_unused_timer(true);
        dma_timer_set_fraction((uint)dma_timer_m, 1, (uint16_t)sample_period);
        setup_channel(dma_ch_m, dma_timer_m, slice_r);
        start_mask |= 1u << dma_ch_m;
    }

    /* Both slices in phase, then both DMA channels at once. */
    pwm_set_counter(slice_l, 0);
    if (slice_r != slice_l) pwm_set_counter(slice_r, 0);
    pwm_set_mask_enabled(pwm_hw->en | (1u << slice_l) | (1u << slice_r));
    dma_start_channel_mask(start_mask);

    wr_pos = (rd_pos() + RING_TARGET) & RING_MASK;
    initialized = true;

    printf("PWM audio: slices %u/%u rate=%lu carrier=%lu Hz wrap=%lu\n",
           slice_l, slice_r, (unsigned long)(sys_clk / sample_period),
           (unsigned long)(sys_clk / wrap_plus_one), (unsigned long)pwm_wrap);
}

/* Append to the ring, keeping the fill level near RING_TARGET. Either
 * `words` (ready CC words) or `samples` (signed 16-bit) is given. */
static void ring_put(const uint32_t *words, const int16_t *samples, int count) {
    if (!initialized || count <= 0) return;

    uint32_t fill = (wr_pos - rd_pos()) & RING_MASK;
    if (fill > RING_MAX) {
        /* The reader ran past us (underrun) or we are far ahead: restart
         * the stream RING_TARGET samples ahead of the reader. */
        wr_pos = (rd_pos() + RING_TARGET) & RING_MASK;
        fill = RING_TARGET;
    }

    bool skip = fill > RING_TARGET + RING_SLACK;                   /* drop one */
    bool dup  = fill + (uint32_t)count < RING_TARGET - RING_SLACK;  /* add one  */

    const int32_t center = (int32_t)pwm_center;
    const int32_t max_lvl = (int32_t)pwm_wrap;
    const int32_t swing = (int32_t)(pwm_wrap >> 1);   /* ~6 dB headroom */

    for (int i = skip ? 1 : 0; i < count; i++) {
        uint32_t w;
        if (words) {
            w = words[i];
        } else {
            int32_t lvl = (((int32_t)samples[i] * swing) >> 15) + center;
            if (lvl < 0) lvl = 0;
            if (lvl > max_lvl) lvl = max_lvl;
            w = ((uint32_t)lvl << 16) | (uint32_t)lvl;
        }
        ring[wr_pos] = w;
        wr_pos = (wr_pos + 1) & RING_MASK;
        last_word = w;
    }
    if (dup) {
        ring[wr_pos] = last_word;
        wr_pos = (wr_pos + 1) & RING_MASK;
    }
}

void pwm_audio_push_samples(const int16_t *buf, int count) {
    ring_put(NULL, buf, count);
}

void pwm_audio_fill_silence(int count) {
    uint32_t silence[64];
    uint32_t w = (pwm_center << 16) | pwm_center;
    for (int i = 0; i < 64; i++) silence[i] = w;
    while (count > 0) {
        int n = count > 64 ? 64 : count;
        ring_put(silence, NULL, n);
        count -= n;
    }
}

void pwm_audio_set_frame_rate(int frame_rate) {
    /* Samples are paced by the DMA timer and the ring absorbs the
     * per-frame variation, so the frame rate no longer matters here. */
    (void)frame_rate;
}
