// Header-only PIO PDM output helper for Arduino-pico (RP2040/RP2350)
// Provides a minimal PIO program and a tiny wrapper to feed 1-bit data.
//
// Program (4 instructions):
//   pull        ; pull 32-bit word from TX FIFO (blocking)
//   out  x, 1   ; shift 1 bit MSB-first into X (consumes one OSR bit)
//   out  pins,1 ; shift 1 bit MSB-first to pins (outputs one bit)
//   jmp  0      ; loop back to address 0 (requires program at offset 0)
//
// Notes:
// - The program consumes 2 OSR bits per loop but only outputs 1 bit.
//   If you use writeBit(), this header packs your bit at position 30 so the
//   emitted pin bit matches the argument. writeWord() pushes raw words.
// - The program uses jmp 0 (absolute). begin() attempts to load at offset 0.
//   If offset 0 is unavailable, begin() will not enable the state machine.
// - FIFO is TX-only as required.
// - Shift-out is MSB-first as required.

#pragma once

#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

// Exclude RP2040. Allow only RP2350 under the Arduino-pico core.
// Some cores define ARDUINO_ARCH_RP2040 for both RP2040 and RP2350 boards.
// We therefore only error when ARDUINO_ARCH_RP2040 is present AND RP2350 is not.
#if defined(ARDUINO_ARCH_RP2040)
#  if !defined(PICO_RP2350)
#    error "PIO_PDM: Only RP2350 supported (RP2040 excluded)."
#  endif
#endif

class PIO_PDM {
public:
    // pio: pio0 or pio1
    // sm:  state machine index [0..3]
    // pin: output GPIO pin
    // freq: target bit output rate (bits per second). One output bit per loop.
    PIO_PDM(PIO pio, uint sm, uint pin, uint freq)
        : pio_(pio), sm_(sm), pin_(pin), freq_(freq), offset_(UINT32_MAX), top_(65535u), sd_accum_(0) {}

    // Upload program, configure pin + SM, set clkdiv, enable SM.
    void begin() {
        // Assemble the minimal program at runtime using SDK encoders.
        static uint16_t insn[4];
        // pull (blocking)
        insn[0] = pio_encode_pull(false, true);
        // out x, 1
        insn[1] = pio_encode_out(pio_x, 1);
        // out pins, 1
        insn[2] = pio_encode_out(pio_pins, 1);
        // jmp 0 (absolute to instruction memory 0)
        insn[3] = pio_encode_jmp(0);

        pio_program prog = {
            .instructions = insn,
            .length = 4,
            .origin = 0
        };

        // Ensure program can be placed at offset 0 due to jmp 0
        if (!pio_can_add_program_at_offset(pio_, &prog, 0)) {
            // Cannot satisfy jmp 0 origin; leave SM disabled.
            return;
        }

        // Load instructions at offset 0
        pio_add_program_at_offset(pio_, &prog, 0);
        offset_ = 0;

        // Configure GPIO for PIO control and as output
        pio_gpio_init(pio_, pin_);
        // State machine configuration
        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_out_pins(&c, pin_, 1);
        // PIO controls the pin direction
        pio_sm_set_consecutive_pindirs(pio_, sm_, pin_, 1, true);

        // Shift out MSB-first; no autopull; threshold 32 (unused here)
        sm_config_set_out_shift(&c, /*shift_right=*/false, /*autopull=*/false, /*pull_threshold=*/32);

        // TX-only FIFO
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

        // Clock divider: one output bit per 4 instructions in the loop
        // (pull, out x,1, out pins,1, jmp 0)
        uint32_t clk_hz = clock_get_hz(clk_sys);
        float div = (float)clk_hz / (float)(freq_ * 4u);
        if (div < 1.0f) div = 1.0f; // guard: minimum divider
        sm_config_set_clkdiv(&c, div);

        // Initialize SM to start of program (offset 0)
        pio_sm_init(pio_, sm_, offset_ + 0, &c);

        // Clear FIFOs and start
        pio_sm_clear_fifos(pio_, sm_);
        pio_sm_set_enabled(pio_, sm_, true);
    }

    // Start a stable 50% duty square wave using DMA feeding the PIO TX FIFO.
    // The frequency on the pin will be approximately freq_/2 (i.e., BITRATE/2).
    // Returns true if DMA started; false if initialization failed.
    bool startStableToggleDMA() {
        if (dma_running_) return true;

        // Ensure SM was configured
        // Pre-fill FIFO to avoid initial underflow
        for (int i = 0; i < 8; ++i) {
            while (pio_sm_is_tx_fifo_full(pio_, sm_)) {}
            pio_sm_put(pio_, sm_, kOneWord);
            while (pio_sm_is_tx_fifo_full(pio_, sm_)) {}
            pio_sm_put(pio_, sm_, kZeroWord);
        }

        // Prepare a tiny 2-word pattern buffer (1 then 0) with 8-byte alignment
        static uint32_t pattern[2] __attribute__((aligned(8))) = {kOneWord, kZeroWord};

        // Claim two channels
        dma_ch0_ = dma_claim_unused_channel(false);
        dma_ch1_ = dma_claim_unused_channel(false);
        if (dma_ch0_ < 0 || dma_ch1_ < 0) {
            // release if partial
            if (dma_ch0_ >= 0) dma_channel_unclaim(dma_ch0_);
            if (dma_ch1_ >= 0) dma_channel_unclaim(dma_ch1_);
            dma_ch0_ = dma_ch1_ = -1;
            return false;
        }

        dma_channel_config c0 = dma_channel_get_default_config(dma_ch0_);
        channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
        channel_config_set_read_increment(&c0, true);
        channel_config_set_write_increment(&c0, false);
        channel_config_set_dreq(&c0, pio_get_dreq(pio_, sm_, true)); // TX DREQ
        // Ring the READ address over 8 bytes (two uint32_t)
        channel_config_set_ring(&c0, /*write=*/false, /*size_bits=*/3);
        channel_config_set_chain_to(&c0, dma_ch1_);

        dma_channel_config c1 = dma_channel_get_default_config(dma_ch1_);
        channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
        channel_config_set_read_increment(&c1, true);
        channel_config_set_write_increment(&c1, false);
        channel_config_set_dreq(&c1, pio_get_dreq(pio_, sm_, true));
        channel_config_set_ring(&c1, /*write=*/false, /*size_bits=*/3);
        channel_config_set_chain_to(&c1, dma_ch0_);

        // Configure both channels
        dma_channel_configure(
            dma_ch0_, &c0,
            &pio_->txf[sm_],   // write address (fixed)
            pattern,           // read address (ringed)
            0xFFFF,            // transfers per chain
            false);

        dma_channel_configure(
            dma_ch1_, &c1,
            &pio_->txf[sm_],
            pattern,
            0xFFFF,
            false);

        // Start DMA
        dma_start_channel_mask((1u << dma_ch0_));
        dma_running_ = true;
        return true;
    }

    // Stop DMA (if running) and release channels
    void stopDMA() {
        if (!dma_running_) return;
        if (dma_ch0_ >= 0) dma_channel_abort(dma_ch0_);
        if (dma_ch1_ >= 0) dma_channel_abort(dma_ch1_);
        if (dma_ch0_ >= 0) dma_channel_unclaim(dma_ch0_);
        if (dma_ch1_ >= 0) dma_channel_unclaim(dma_ch1_);
        dma_ch0_ = dma_ch1_ = -1;
        dma_running_ = false;
    }

    // Optional: Set the full-scale value used by sigma-delta in write(value).
    inline void setTop(uint32_t top) { top_ = top ? top : 1u; }
    inline uint32_t top() const { return top_; }

    // Sigma-delta write: accepts a value in [0..top()], emits one PDM bit.
    // Non-blocking except when TX FIFO is full (then it spins until space).
    inline void write(uint32_t value) {
        if (value > top_) value = top_;
        bool bit = (sd_accum_ >= 0);
        // Update integrator: target minus realized output
        sd_accum_ += (int64_t)value - (bit ? (int64_t)top_ : 0);
        writeBit(bit);
    }

    // 16-bit convenience overload (recommended public form)
    inline void write(uint16_t value) { write((uint32_t)value); }

    // Push a single output bit. Blocks only if TX FIFO is full.
    // Packs the bit at position 30 so that after 'out x,1' (discard) and
    // 'out pins,1' (emit), the pin reflects 'b'.
    inline void writeBit(bool b) {
        uint32_t w = b ? (1u << 30) : 0u;
        while (pio_sm_is_tx_fifo_full(pio_, sm_)) {}
        pio_sm_put(pio_, sm_, w);
    }

    // Push a 32-bit word to the TX FIFO. Blocks only if TX FIFO is full.
    // Note: With this program, only one bit (bit 30) of each word will reach
    // the pin per loop iteration. If you need to emit 32 packed bits, feed
    // them via repeated writeBit() calls or pre-pack to match the program.
    inline void writeWord(uint32_t w) {
        while (pio_sm_is_tx_fifo_full(pio_, sm_)) {}
        pio_sm_put(pio_, sm_, w);
    }

private:
    PIO pio_;
    uint sm_;
    uint pin_;
    uint freq_;
    uint offset_;
    uint32_t top_;
    int64_t sd_accum_;
    int dma_ch0_ = -1;
    int dma_ch1_ = -1;
    bool dma_running_ = false;

    static constexpr uint32_t kOneWord  = (1u << 30);
    static constexpr uint32_t kZeroWord = 0u;
};
