#include <Arduino.h>
#include <PIO_PDM.h>
#include <math.h>

static constexpr uint PDM_PIN   = 2;
static constexpr uint SM_IDX    = 0;
static constexpr uint BITRATE   = 500000;
static constexpr float TONE_HZ  = 3000.0f;

PIO_PDM pdm(pio1, SM_IDX, PDM_PIN, BITRATE);

void setup() {
  pdm.begin();
}

void loop() {
  static float phase = 0.0f;
  const float two_pi = 6.283185307179586f;
  const float dphi   = two_pi * (TONE_HZ / (float)BITRATE);

  // Generate 0..65535 sample
  float s = sinf(phase) * 0.5f + 0.5f;  // [0..1]
  uint16_t v = (uint16_t)(s * 65535.0f + 0.5f);
  pdm.write(v);

  phase += dphi;
  if (phase >= two_pi) phase -= two_pi;
}
