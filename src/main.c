/**
  ******************************************************************************
  * @file    main.c
  * @author  Andrew Dile, Abhi Annigeri, Daniel Wang, William Lee
  * @date    November 4 2024
  * @brief   ECE 362 Snake Game
  ******************************************************************************
*/

#include "header.h"
// Game State Definitions
#define IDLE 0
#define PLAYING 1
#define GAME_OVER 2

// Joystick Direction Definitions
#define NEUTRAL 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4

// Game Object Identifiers for `gameboard`
#define EMPTY 0 // Empty cell
#define SNACK 1 // Cell with a snack
#define SNAKE_HEAD 2 // Head of the snake
#define SNAKE_BODY 3 // Body of the snake

// Game Board Size (Adjust as needed)
#define NUM_X_CELLS 20 // Number of cells horizontally
#define NUM_Y_CELLS 20 // Number of cells vertically

// Speed and Difficulty
#define INITIAL_SPEED 500 // Initial delay in milliseconds
#define SPEED_INCREMENT 50 // Speed up after each snack
#define MAX_SPEED 100 // Minimum delay in milliseconds


// variables for game state handling
int8_t gameState = IDLE;
int8_t lastGameState = IDLE;

// gameboard is 2D array of gametile structures, value of tile updated on snake movement
uint8_t gameboard[NUM_X_CELLS][NUM_Y_CELLS] = {0}; // value determines what is displayed in that cell (0 = blank, 1 = snack, 2 = head facing left, etc.)

// array to store snake
segment snake[NUM_X_CELLS * NUM_Y_CELLS]; // not dynamically stored to avoid fragmentation and leaks, and to ensure program doesn't run out of space during gameplay

// snake starting length
int snakeLength = 1;

// starting speed
int gameSpeed = INITIAL_SPEED;

// variable for joystick direction
int8_t joystickDirection = NEUTRAL;

// flag for some code to only run on first cycle
bool initialized = false;


void init_gpio() {
    // Enable clocks for GPIOA and GPIOB
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);

    // Configure PA0 and PA1 for joystick (analog input mode)
    GPIOA->MODER |= (GPIO_MODER_MODER0 | GPIO_MODER_MODER1); // Set PA0 and PA1 to analog mode (11)

    // Configure PB0 and PB1 as general-purpose outputs for LED feedback
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1); // Clear mode bits
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0); // Set to output mode (01)

    // Configure GPIO speed and pull-up/pull-down settings as necessary
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1); // High speed for LEDs
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1); // No pull-up or pull-down for LEDs
}

void init_adc() {
    // Enable clock for ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
}

int read_joystick_x() {

}

int read_joystick_y() {

}

// Determine joystick direction based on ADC input
int8_t getJoystickDirection(void) {
    int x = read_joystick_x();
    int y = read_joystick_y();

    if (x > 200) return RIGHT;
    else if (x < 50) return LEFT;
    else if (y > 200) return UP;
    else if (y < 50) return DOWN;
    else return NEUTRAL;
}

// Update joystick direction from ADC input
void updateJoystickDirection(void) {
  joystickDirection = getJoystickDirection(); // get joystick direction from ADC
}

void moveSnake(void) {
  // Create a new head position based on joystickDirection
  segment newHead = snake[0];
  switch (joystickDirection) {
    case UP:    newHead.y--; break;
    case DOWN:  newHead.y++; break;
    case LEFT:  newHead.x--; break;
    case RIGHT: newHead.x++; break;
  }

  // Check if new head collides with snack, wall, or itself
  if (gameboard[newHead.x][newHead.y] == SNACK) {
    snakeLength++;  // Increase snake length
    gameSpeed -= SPEED_INCREMENT;  // Increase speed
    generateSnack();
  } else {
    // Move snake by shifting all segments back by one
    for (int i = snakeLength - 1; i > 0; i--) {
      snake[i] = snake[i - 1];
    }
  }
  snake[0] = newHead;
}

// Check for collisions with boundaries or snake itself
void checkCollisions(void) {
  if (snake[0].x < 0 || snake[0].x >= NUM_X_CELLS || snake[0].y < 0 || snake[0].y >= NUM_Y_CELLS ||
      gameboard[snake[0].x][snake[0].y] == SNAKE_BODY) {
    gameState = GAME_OVER;
    // Trigger game over sequence
    handleGameOver();
  }
}

// Generate snack in a random location on the game board
void generateSnack(void) {
  int snackX = rand() % NUM_X_CELLS;
  int snackY = rand() % NUM_Y_CELLS;
  gameboard[snackX][snackY] = SNACK;
}

// Update game display to reflect snake and snack positions
void updateGameDisplay(void) {
  clearDisplay();
  for (int i = 0; i < snakeLength; i++) {
    drawSegment(snake[i].x, snake[i].y); // draws each snake segment
  }
  drawSnack(); // draw snack position
}

void init_pwm() {
    // Enable clock for Timer 3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure Timer 3 for PWM on Channel 1
    TIM3->PSC = 7999; // Set prescaler to divide clock for desired frequency
    TIM3->ARR = 999;  // Auto-reload for desired PWM period
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1 on Channel 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload for channel 1
    TIM3->CCER |= TIM_CCER_CC1E; // Enable output for channel 1

    // Set duty cycle
    TIM3->CCR1 = 500; // 50% duty cycle initially

    // Start Timer 3
    TIM3->CR1 |= TIM_CR1_CEN;
}

void play_sound(int frequency, int duration) {
    TIM3->ARR = frequency; // Set frequency
    TIM3->CCR1 = frequency / 2; // Set 50% duty cycle
    TIM3->CR1 |= TIM_CR1_CEN; // Enable timer

    // Delay to play sound for specified duration
    for (volatile int i = 0; i < duration * 1000; i++) {} // Simple delay loop

    TIM3->CR1 &= ~TIM_CR1_CEN; // Disable timer after sound duration
}

void init_timer() {
    // Enable clock for Timer 2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure Timer 2
    TIM2->PSC = 7999; // Prescaler to create a 1 ms tick
    TIM2->ARR = 499; // Auto-reload value for 500 ms interval (adjustable for speed)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN; // Enable timer

    // Enable Timer 2 interrupt in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
}


// Timer-based delay for game speed control
void speedDelay(int ms) {
    TIM2->ARR = ms; // Set delay
    TIM2->CNT = 0; // Reset counter

    TIM2->CR1 |= TIM_CR1_CEN; // Enable timer
    while (!(TIM2->SR & TIM_SR_UIF)) {} // Wait for update interrupt flag
    TIM2->SR &= ~TIM_SR_UIF; // Clear the flag
    TIM2->CR1 &= ~TIM_CR1_CEN; // Stop timer
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Check for update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF; // Clear the flag

        // Game update code goes here (move snake, check collisions)
        moveSnake();
        checkCollisions();
        updateGameDisplay();
    }
}



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

    // start game with a snack
    generateSnack();
  }

  while(1) {
    updateJoystickDirection();
    moveSnake();
    checkCollisions();
    updateLCDDisplay();
    updateGameDisplay();
    speedDelay(gameSpeed); 
  }


  // update displays
  updateLCDDisplay();
  updateGameDisplay();
}