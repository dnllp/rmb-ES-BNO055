#pragma once
// Un módulo BTS7960 (IBT-2): PWM a 20 kHz, sentido y freno activo. (Los pines IS no se usan.)
#include <Arduino.h>

class Bts7960 {
 public:
  struct Pins {
    int rpwm, lpwm, en;
    uint8_t chR, chL;
  };

  void begin(const Pins &p, uint32_t freqHz, uint8_t bits, bool invert) {
    p_ = p;
    invert_ = invert;
    maxDuty_ = (1u << bits) - 1;
    pinMode(p_.en, OUTPUT);
    digitalWrite(p_.en, LOW);  // apagado hasta que se pida
    ledcSetup(p_.chR, freqHz, bits);
    ledcSetup(p_.chL, freqHz, bits);
    ledcAttachPin(p_.rpwm, p_.chR);
    ledcAttachPin(p_.lpwm, p_.chL);
    ledcWrite(p_.chR, 0);
    ledcWrite(p_.chL, 0);
  }

  // duty en [−1, 1]. Positivo = avanzar.
  void setDuty(float duty) {
    if (duty > 1) duty = 1;
    if (duty < -1) duty = -1;
    if (invert_) duty = -duty;
    uint32_t d = (uint32_t)(fabsf(duty) * maxDuty_ + 0.5f);
    digitalWrite(p_.en, HIGH);
    if (duty >= 0) {
      ledcWrite(p_.chL, 0);
      ledcWrite(p_.chR, d);
    } else {
      ledcWrite(p_.chR, 0);
      ledcWrite(p_.chL, d);
    }
    duty_ = invert_ ? -duty : duty;
    braking_ = false;
  }

  // Freno activo: los dos lados bajos conducen (RPWM = LPWM = 0 con EN alto).
  void brake() {
    ledcWrite(p_.chR, 0);
    ledcWrite(p_.chL, 0);
    digitalWrite(p_.en, HIGH);
    duty_ = 0;
    braking_ = true;
  }

  // Desactiva el puente (rueda libre). Lo usa el paro de emergencia tras frenar.
  void disable() {
    ledcWrite(p_.chR, 0);
    ledcWrite(p_.chL, 0);
    digitalWrite(p_.en, LOW);
    duty_ = 0;
    braking_ = false;
  }

  float duty() const { return duty_; }
  bool braking() const { return braking_; }

 private:
  Pins p_{};
  bool invert_ = false;
  uint32_t maxDuty_ = 1023;
  float duty_ = 0;
  bool braking_ = false;
};
