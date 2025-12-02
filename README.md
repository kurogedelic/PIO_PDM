# PIO_PDM

Minimal PIO 1‑bit PDM output library for Arduino‑pico (RP2350).

RP2040 is intentionally excluded at compile time.

## Overview

- Outputs a continuous 1‑bit stream using a PIO state machine.
- MSB‑first shifting, TX‑only FIFO, pin direction set by PIO.
- Bit rate is set by the constructor `bitrate` argument (bits per second).

## API

```cpp
#include <PIO_PDM.h>

// Construct
PIO_PDM pdm(pio1, /*sm_index*/ 0, /*pin*/ 2, /*bitrate*/ 500000);

// Start PIO state machine
pdm.begin();

// Feed values (0..65535). Library converts to 1‑bit PDM (ΔΣ)
pdm.write((uint16_t)32768);
```

## License

MIT © @kurogedelic
