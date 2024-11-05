/**
  ******************************************************************************
  * @file    main.c
  * @author  Andrew Dile, 
  * @date    November 4 2024
  * @brief   ECE 362 Snake Game
  ******************************************************************************
*/

#include "header.h"

// gameboard is 2D array of gametile structures, value of tile updated on snake movement
uint8_t gameboard[NUM_X_CELLS][NUM_Y_CELLS] = {0}; // value determines what is displayed in that cell (0 = blank, 1 = snack, 2 = head facing left, etc.)

// array to store snake
segment snake[NUM_X_CELLS * NUM_Y_CELLS]; // not dynamically stored to avoid fragmentation and leaks, and to ensure program doesn't run out of space during gameplay

// flag for some code to only run on first cycle
bool initialized = false;

int main(void) {
  if (!initialized) {
    // set up game display (and SD interface) for communication
    setupGameDisplay();

    // get high scores from SD memory
    readMemory();

    // set LCD display with high scores
    setupLCDDisplay();

    // set up joystick for ADC readings
    setupJoystick();

    // set flag so this if statement only runs on first loop
    initialized = true;
  }

  // update the snake's head's direction based on the joystick reading
  snake[0].direction = readJoystick();

  // update displays
  updateLCDDisplay();
  updateGameDisplay();
}