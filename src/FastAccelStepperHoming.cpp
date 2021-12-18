#include "FastAccelStepperHoming.h"

void FastAccelStepperHoming::init(FastAccelStepper* stepper) {
	_stepper = stepper;
}

void FastAccelStepperHoming::setLimitPin(uint8_t limitPin, uint8_t mode){
	_limitPin = limitPin;
	_limitPinSet = true;
	pinMode(limitPin, mode);
	_limitPinState = digitalRead(limitPin);
	attachInterrupt(digitalPinToInterrupt(limitPin), std::bind(&FastAccelStepperHoming::limit_isr, this), CHANGE);
}

void FastAccelStepperHoming::setHomingSpeedInUs(uint32_t homing_step_us) {
	_homing_step_us = homing_step_us;
}

void FastAccelStepperHoming::setHomingPulloffSteps(uint32_t homing_pulloff_steps) {
    _homing_pulloff_steps = homing_pulloff_steps;
}
void FastAccelStepperHoming::setInvertLimitPin(bool invertLimitPin) {
	_invertLimitPin = invertLimitPin;
}

void FastAccelStepperHoming::home() {
	if(_limitPinSet) {
		_initial_step_us = _stepper->getSpeedInUs();
		
		if(_homing_step_us != 0){
			_stepper->setSpeedInUs(_homing_step_us);
		}
		
		if(_limitPinState == _invertLimitPin) {
			//We are some distance away from the limit sensor
			//So try to move towards it, assuming it is in the negative direction
			_homingHome = true;
			_stepper->moveByAcceleration(-_stepper->getAcceleration());
		} else {
			//We are near the limit sensor
			_homingHome = false;
			_stepper->moveByAcceleration(_stepper->getAcceleration());
		}
		
		_isHoming = true;
	}
}

bool FastAccelStepperHoming::isHomingComplete(){
	if(!_isHoming && !_stepper->isRunning()){
		return true;
	}
		
	return false;
}
void ARDUINO_ISR_ATTR FastAccelStepperHoming::limit_isr() {
	bool pinState = digitalRead(_limitPin); // Save current state so we can compare with the last saved pin state to ignore false triggers
	
	if(_isHoming && _limitPinState != pinState) {
		if((!pinState == _invertLimitPin && _homingHome) || (pinState == _invertLimitPin && !_homingHome)){
			_stepper->forceStopAndNewPosition(0);
			_stepper->setSpeedInUs(_initial_step_us);
			_stepper->move(_homing_pulloff_steps);
		}
		
		_isHoming = false; //Note, the stepper may still be in motion
	}
	
	_limitPinState = pinState;
}