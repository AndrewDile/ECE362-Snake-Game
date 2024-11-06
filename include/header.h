/**
  ******************************************************************************
  * @file    header.h
  * @author  Andrew Dile, Abhi Annigeri, Daniel Wang, William Lee
  * @date    November 4 2024
  * @brief   ECE 362 Snake Game
  ******************************************************************************
*/

//////////// PIN MAP ////////////
// 
//  PA1 as ADC1 -- JoystickX
//  PA2 as ADC2 -- JoystickY
//  PA3 as GPIO -- JoystickSw
//
/////////////////////////////////

// global includes
#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

// number of cells on the gameboard, can be changed but will drastically affect memory game takes to play
#define NUM_X_CELLS 25
#define NUM_Y_CELLS 25

// direction constants for code readability and standardization
#define NEUTRAL -1
#define RIGHT 0
#define LEFT 1
#define UP 2
#define DOWN 3

// game state constants for code readability and standardization
#define IDLE 0
#define RUNNING 1
#define GAMELOST 2
#define GAMEWON 3

// constants for tile pieces
#define EMPTY 0
#define SNACK 1
#define HEAD_LEFT 2
#define HEAD_RIGHT 3
#define HEAD_UP 4
#define HEAD_DOWN 5
#define SEGMENT_VER 6
#define SEGMENT_HOR 7
#define BEND_UP_RIGHT 8
#define BEND_UP_LEFT 9
#define BEND_DOWN_RIGHT 10
#define BEND_DOWN_LEFT 11
#define BEND_RIGHT_UP BEND_UP_RIGHT
#define BEND_RIGHT_DOWN BEND_DOWN_RIGHT
#define BEND_LEFT_UP BEND_UP_LEFT
#define BEND_LEFT_DOWN BEND_DOWN_LEFT

// each segment of the snake needs a position and a direction value
struct segment {
  uint8_t x; // horizontal gametile position -- left = 0
  uint8_t y; // vertical gametile position -- bottom = 0
  uint8_t direction; // direction current segment (or head) is facing
};

// defines the structure as a data type, for easier later use
typedef struct segment segment;

// exteral variables declared so other files to access
extern int8_t gameState;
extern int8_t lastGameState;
extern int8_t joystickDirection;
extern uint8_t gameboard[NUM_X_CELLS][NUM_Y_CELLS];
extern segment snake[NUM_X_CELLS * NUM_Y_CELLS];

// function declarations
void readMemory();
void setupLCDDisplay();
void updateLCDDisplay();
void setupJoystick();
void movementLogic();
void setupPWM();
void playSound(uint8_t song);
void setupGameDisplay();
void updateGameDisplay();