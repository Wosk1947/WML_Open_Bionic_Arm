

#ifndef DOUBLECLICKER_H
#define DOUBLECLICKER_H

#include "Arduino.h"

class DoubleClicker {
  public:
	  enum ClickerState {
		  HOLD,
		  BEGIN,
		  FIRST_DOWN,
		  FIRST_UP,
		  SECOND_DOWN,
		  SECOND_UP
	  };
	  enum ButtonState {
		  NO,
		  CLICK,
		  DOUBLE_CLICK,
		  BUTTON_HOLD
	  };
	  ClickerState state = BEGIN;
	  ButtonState buttonState = NO;
	  int currentButtonState;
	DoubleClicker(int pin, int intervalMS, int holdMS, int pressedState);
	bool update();
	ButtonState getState();
  private:
	 
	int pin;
	int intervalMS;
	int holdMS;
	int pressedState;
	long lastClickTime;
	long lastHoldTime;
	long transitionTime = 50;
	long lastButtonChangeTime;
};

#endif
