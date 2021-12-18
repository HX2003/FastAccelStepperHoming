#ifndef _FastAccelStepperHoming_H_
#define _FastAccelStepperHoming_H_

#include <Arduino.h>
#undef max
#undef min
#include <FunctionalInterrupt.h>
#include <FastAccelStepper.h>

class FastAccelStepperHoming {
 public:
  void init(FastAccelStepper* stepper);
  void setLimitPin(uint8_t limitPin, uint8_t mode = INPUT);
  void setHomingSpeedInUs(uint32_t homing_step_us);
  void setHomingPulloffSteps(uint32_t homing_pulloff_steps);
  void setInvertLimitPin(bool invertLimitPin);
  void ARDUINO_ISR_ATTR limit_isr();
  void home();
  bool isHomingComplete();
 
 private:
  FastAccelStepper* _stepper;
  uint32_t _initial_step_us;
  uint32_t _homing_step_us = 0;
  uint32_t _homing_pulloff_steps = 100;
  bool _limitPinSet = false;
  volatile uint8_t _limitPin;
  volatile bool _invertLimitPin = false;
  volatile bool _limitPinState;
  volatile bool _isHoming = false;
  volatile bool _homingHome;
};

#endif
