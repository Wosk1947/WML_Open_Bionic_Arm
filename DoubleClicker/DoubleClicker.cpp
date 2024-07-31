
#include "DoubleClicker.h"
#include "Arduino.h"


DoubleClicker::DoubleClicker(int pin, int intervalMS, int holdMS, int pressedState){
	this->pin = pin;
	this->intervalMS = intervalMS;
	this->holdMS = holdMS;
	this->pressedState = pressedState;
}

bool DoubleClicker::update(){
	long currentTime = static_cast<long>(millis());
	currentButtonState = digitalRead(pin);
	if (buttonState == CLICK || buttonState == DOUBLE_CLICK) {
		buttonState = NO;
	}
	switch (state) {
		case HOLD:
			if (currentButtonState != pressedState) {
				state = BEGIN;
				buttonState = NO;
				lastButtonChangeTime = currentTime;
			}
			break;
		case BEGIN:
			if (currentTime <= lastButtonChangeTime + transitionTime) {
				break;
			}
			if (currentButtonState == pressedState) {
				state = FIRST_DOWN;
				lastHoldTime = currentTime;
				lastButtonChangeTime = currentTime;
			}
			break;
		case FIRST_DOWN:
			if (currentTime <= lastButtonChangeTime + transitionTime) {
				break;
			}
			if (currentButtonState != pressedState) {
				state = FIRST_UP;
				lastClickTime = currentTime;
				lastButtonChangeTime = currentTime;
				break;
			}
			if (currentTime > lastHoldTime + holdMS) {
				state = HOLD;
				buttonState = BUTTON_HOLD;
			}
			break;
		case FIRST_UP:
			if (currentTime <= lastButtonChangeTime + transitionTime) {
				break;
			}
			if (currentTime > lastClickTime + intervalMS) {
				state = BEGIN;
				buttonState = CLICK;
				break;
			}
			if (currentButtonState == pressedState) {
				state = SECOND_DOWN;
				lastHoldTime = currentTime;
				lastButtonChangeTime = currentTime;
			}
			break;
		case SECOND_DOWN:
			if (currentTime <= lastButtonChangeTime + transitionTime) {
				break;
			}
			if (currentButtonState != pressedState) {
				state = BEGIN;
				lastButtonChangeTime = currentTime;
				buttonState = DOUBLE_CLICK;
			}
			if (currentTime > lastHoldTime + holdMS) {
				state = HOLD;
				buttonState = BUTTON_HOLD;
			}
			break;
	}
	return false;
}

DoubleClicker::ButtonState DoubleClicker::getState() {
	return buttonState;
}
