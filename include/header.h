/**
  ******************************************************************************
  * @file    header.h
  * @author  Andrew Dile, Abhi Annigeri, Daniel Wang, William Lee
  * @date    November 4 2024
  * @brief   ECE 362 Snake Game
  ******************************************************************************
*/

// global includes
#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// number of cells on the gameboard, can be changed but will drastically affect memory game takes to play
#define NUM_X_CELLS 25
#define NUM_Y_CELLS 25

// direction constants for code readability and standardization
#define RIGHT 0
#define LEFT 1
#define UP 2
#define DOWN 3

// each segment of the snake needs a position and a direction value
struct segment {
  uint8_t x; // horizontal gametile position -- left = 0
  uint8_t y; // vertical gametile position -- bottom = 0
  uint8_t direction; // direction current segment (or head) is facing
};

// defines the structure as a data type, for easier later use
typedef struct segment segment;

// exteral variables declared so other files to access
extern uint8_t gameboard[NUM_X_CELLS][NUM_Y_CELLS];
extern segment snake[NUM_X_CELLS * NUM_Y_CELLS];

// function declarations
void readMemory();
void setupLCDDisplay();
void updateLCDDisplay();
void setupJoystick();
uint8_t readJoystick();
void movementLogic();
void setupPWM();
void playSound(uint8_t song);
void setupGameDisplay();
void updateGameDisplay();