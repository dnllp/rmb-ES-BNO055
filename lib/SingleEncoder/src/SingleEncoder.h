#pragma once
// Encoder de UN canal: el PCNT cuenta los pulsos (con filtro de glitches) y una
// interrupción guarda la marca de tiempo de cada flanco de subida para medir el
// periodo entre pulsos. El signo NO lo da el sensor: lo fija Motion con setDirection().
#include <Arduino.h>
#include <driver/pcnt.h>
#include <esp_timer.h>

class SingleEncoder {
 public:
  bool begin(int pin, pcnt_unit_t unit, bool bothEdges, uint16_t filterNs, uint32_t stopTimeoutUs) {
    pin_ = pin;
    unit_ = unit;
    edgesPerPulse_ = bothEdges ? 2 : 1;
    stopTimeoutUs_ = stopTimeoutUs;

    pcnt_config_t c = {};
    c.pulse_gpio_num = pin;
    c.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    c.channel = PCNT_CHANNEL_0;
    c.unit = unit;
    c.pos_mode = PCNT_COUNT_INC;
    c.neg_mode = bothEdges ? PCNT_COUNT_INC : PCNT_COUNT_DIS;
    c.lctrl_mode = PCNT_MODE_KEEP;
    c.hctrl_mode = PCNT_MODE_KEEP;
    c.counter_h_lim = H_LIM;
    c.counter_l_lim = -1;
    if (pcnt_unit_config(&c) != ESP_OK) return false;

    // Filtro en ciclos de APB (80 MHz), máximo 1023 (~12.8 µs)
    uint32_t cycles = (uint32_t)filterNs * 80u / 1000u;
    if (cycles > 1023) cycles = 1023;
    pcnt_set_filter_value(unit, (uint16_t)cycles);
    pcnt_filter_enable(unit);

    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    static bool isrService = false;
    if (!isrService) {
      pcnt_isr_service_install(0);
      isrService = true;
    }
    pcnt_isr_handler_add(unit, &SingleEncoder::overflowIsr, this);
    pcnt_counter_resume(unit);

    attachInterruptArg(pin, &SingleEncoder::edgeIsr, this, RISING);
    lastRaw_ = readRaw();
    return true;
  }

  // +1 = la rueda avanza, −1 = retrocede. Solo cambiarlo con la rueda quieta (isStill).
  void setDirection(int8_t dir) { dir_ = dir >= 0 ? 1 : -1; }
  int8_t direction() const { return dir_; }

  // Llamar en cada ciclo de control. Devuelve el incremento con signo, en cuentas.
  int32_t update() {
    int64_t raw = readRaw();
    int32_t d = (int32_t)(raw - lastRaw_);
    lastRaw_ = raw;
    signedTotal_ += (int64_t)dir_ * d;
    rawTotal_ = raw;

    uint64_t sum;
    uint32_t n, lastP;
    int64_t last;
    portENTER_CRITICAL(&mux_);
    sum = sumPeriod_;
    n = nPeriod_;
    sumPeriod_ = 0;
    nPeriod_ = 0;
    last = lastEdgeUs_;
    lastP = lastPeriodUs_;
    portEXIT_CRITICAL(&mux_);

    int64_t now = esp_timer_get_time();
    int64_t since = now - last;
    if (last == 0 || since > (int64_t)stopTimeoutUs_ || lastP == 0) {
      pulsesPerSec_ = 0;
    } else {
      // Promedio de los periodos de este ciclo; si ya pasó más tiempo sin pulso,
      // la rueda va más lenta que eso.
      float T = n ? (float)sum / n : (float)lastP;
      if ((float)since > T) T = (float)since;
      pulsesPerSec_ = 1e6f / T;
    }
    return (int32_t)dir_ * d;
  }

  // Cuentas por segundo con signo (en las mismas unidades que CUENTAS_POR_VUELTA)
  float ticksPerSecond() const { return dir_ * pulsesPerSec_ * edgesPerPulse_; }
  int64_t signedTotal() const { return signedTotal_; }
  int64_t rawTotal() const { return rawTotal_; }

  uint32_t usSinceLastPulse() const {
    int64_t last;
    portENTER_CRITICAL(&mux_);
    last = lastEdgeUs_;
    portEXIT_CRITICAL(&mux_);
    if (last == 0) return UINT32_MAX;
    int64_t d = esp_timer_get_time() - last;
    return d > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)d;
  }
  bool isStill(uint32_t ms) const { return usSinceLastPulse() >= ms * 1000u; }

 private:
  static constexpr int16_t H_LIM = 30000;
  static constexpr uint32_t MIN_PERIOD_US = 300;  // más rápido que esto es ruido (>1.5 m/s)

  int64_t readRaw() {
    int16_t c = 0;
    int32_t o1, o2;
    do {
      o1 = overflows_;
      pcnt_get_counter_value(unit_, &c);
      o2 = overflows_;
    } while (o1 != o2);
    return (int64_t)o1 * H_LIM + c;
  }

  static void IRAM_ATTR overflowIsr(void *arg) {
    auto *self = static_cast<SingleEncoder *>(arg);
    self->overflows_ = self->overflows_ + 1;
  }

  static void IRAM_ATTR edgeIsr(void *arg) {
    auto *self = static_cast<SingleEncoder *>(arg);
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&self->mux_);
    if (self->lastEdgeUs_ != 0) {
      uint32_t p = (uint32_t)(now - self->lastEdgeUs_);
      if (p < MIN_PERIOD_US) {  // glitch: se ignora
        portEXIT_CRITICAL_ISR(&self->mux_);
        return;
      }
      self->lastPeriodUs_ = p;
      self->sumPeriod_ += p;
      self->nPeriod_++;
    }
    self->lastEdgeUs_ = now;
    portEXIT_CRITICAL_ISR(&self->mux_);
  }

  int pin_ = -1;
  pcnt_unit_t unit_ = PCNT_UNIT_0;
  uint8_t edgesPerPulse_ = 1;
  uint32_t stopTimeoutUs_ = 100000;
  volatile int8_t dir_ = 1;
  volatile int32_t overflows_ = 0;
  int64_t lastRaw_ = 0;
  int64_t rawTotal_ = 0;
  int64_t signedTotal_ = 0;
  float pulsesPerSec_ = 0;

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile int64_t lastEdgeUs_ = 0;
  volatile uint32_t lastPeriodUs_ = 0;
  volatile uint64_t sumPeriod_ = 0;
  volatile uint32_t nPeriod_ = 0;
};
