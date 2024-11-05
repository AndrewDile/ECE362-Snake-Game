/**
  ******************************************************************************
  * @file    support.c
  * @author  Andrew Dile, Abhi Annigeri, Daniel Wang, William Lee
  * @date    November 4 2024
  * @brief   ECE 362 Snake Game
  ******************************************************************************
*/

#include "header.h"

// access memory via SPI to SD interface to get high score memory
void readMemory() {

}

// set up LCD display to be communicated with
void setupLCDDisplay() {

}

// function that updates LCD display
void updateLCDDisplay() {

}

// function to initialize ADC for joystick readings
void setupJoystick() {

}

// read joystick via ADC and return a direction value
uint8_t readJoystick() {

}

// iterates over snake and updates segments and gameboard, maybe calls playSound based on what is happening?
void movementLogic() {

}

// set up PWM for use for LED and buzzer
void setupPWM() {

}

// plays song based on value, maybe updates a flag once it finishes a song?
void playSound(uint8_t song) {

}

// set LED color based on value, should be able to pull pretty much verbatum from PWM lab
void setLED(uint8_t value) {

}

// set up game display (and maybe SD interface?) to be communicated with
void setupGameDisplay() {

}

// update the game display based on current gameboard values
void updateGameDisplay() {

}

// IRQ Handler that calls movementLogic() (TIM3 arbitrarily chosen, can be changed)
void TIM3_IRQHandler() {
  
}