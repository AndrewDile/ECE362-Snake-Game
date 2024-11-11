/**
  ******************************************************************************
  * @file    support.c
  * @author  Andrew Dile, Abhi Annigeri, Daniel Wang, William Lee
  * @date    November 4 2024
  * @brief   ECE 362 Snake Game
  ******************************************************************************
*/

#include "header.h"
#define MAX_SNAKE_LENGTH (NUM_X_CELLS * NUM_Y_CELLS)
#define INITIAL_SNAKE_LENGTH 3
#define INITIAL_SNAKE_SPEED 500  // milliseconds between moves
#define MIN_SNAKE_SPEED 100      // maximum speed (minimum delay)
#define SPEED_INCREASE 25        // ms faster per snack eaten

// Snake segment structure (if not already defined in header)
typedef struct {
    uint8_t x;
    uint8_t y;
} segment;

// Global variables (if not already defined)
extern uint8_t snakeLength;
extern uint32_t snakeSpeed;  // in milliseconds
extern bool gameOver;


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
  // enable RCC clocks, port A, and pins
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER |= 0xF;

  // set resolution to 6 bits
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;
  ADC1->CFGR1 |= ADC_CFGR1_RES_1;

  // configure ADC to only do a conversion after the last value has been read
  ADC1->CFGR1 |= ADC_CFGR1_WAIT;

  // select channels 0 and 1 (PA0 for JoystickX, PA1 for JoystickY)
  ADC1->CHSELR = 0x3;

  // enable end of conversion interrupt
  ADC1->IER |= ADC_IER_EOCIE;

  // enable ADC peripheral and wait for it to be ready
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY));

  // start conversion
  ADC1->CR |= ADC_CR_ADSTART;

  // enable ADC interrupt in NVIC
  NVIC_EnableIRQ(ADC1_IRQn);
}

// IRQ Handler when ADC conversion finishes
void ADC1_IRQHandler() {
  // initialize temp variables
  int8_t x = -1;
  int8_t y = -1;

  // read values sequentially
  if (ADC1->ISR & ADC_ISR_EOC) {
    if(x == -1) {
      x = ADC1->DR;
    } else {
      y = ADC1->DR;

      // set joystickDirection based on readings
      if (x < 10) joystickDirection = LEFT;
      else if (x > 50) joystickDirection = RIGHT;
      else if (y < 10) joystickDirection = DOWN;
      else if (y > 50) joystickDirection = UP;
      else joystickDirection = NEUTRAL;

      // start new ADC conversion
      ADC1->CR |= ADC_CR_ADSTART;
    }
  }
}

void initializeSnake() {
    // Start snake in middle of board
    uint8_t startX = NUM_X_CELLS / 2;
    uint8_t startY = NUM_Y_CELLS / 2;
    
    snakeLength = INITIAL_SNAKE_LENGTH;
    snakeSpeed = INITIAL_SNAKE_SPEED;
    gameOver = false;
    
    // Initialize snake segments
    for(int i = 0; i < snakeLength; i++) {
        snake[i].x = startX - i;  // Snake starts horizontally
        snake[i].y = startY;
        gameboard[startX - i][startY] = (i == 0) ? 2 : 3;  // 2 for head, 3 for body
    }
    
    // Generate first snack
    generateSnack();
}

void setupMovementTimer() {
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    // Set prescaler for 1ms resolution
    // Assuming 48MHz system clock
    TIM3->PSC = 47999;  // 48MHz/48000 = 1kHz
    
    // Set initial period to snakeSpeed (in ms)
    TIM3->ARR = INITIAL_SNAKE_SPEED;
    
    // Enable update interrupt
    TIM3->DIER |= TIM_DIER_UIE;
    
    // Enable timer
    TIM3->CR1 |= TIM_CR1_CEN;
    
    // Enable TIM3 interrupt in NVIC
    NVIC_EnableIRQ(TIM3_IRQn);
}

// Generate new snack at random position
void generateSnack() {
    uint8_t x, y;
    bool validPosition;
    
    do {
        validPosition = true;
        
        // Generate random position
        x = rand() % NUM_X_CELLS;
        y = rand() % NUM_Y_CELLS;
        
        // Check if position is occupied by snake
        for(int i = 0; i < snakeLength; i++) {
            if(snake[i].x == x && snake[i].y == y) {
                validPosition = false;
                break;
            }
        }
    } while(!validPosition);
    
    // Place snack on gameboard
    gameboard[x][y] = 1;  // 1 represents snack
}

// iterates over snake and updates segments and gameboard, maybe calls playSound based on what is happening?
void movementLogic() {
   if(gameOver) return;
    
    // Calculate new head position
    uint8_t newHeadX = snake[0].x;
    uint8_t newHeadY = snake[0].y;
    
    // Update position based on joystick direction
    switch(joystickDirection) {
        case UP:
            newHeadY = (newHeadY + 1) % NUM_Y_CELLS;
            break;
        case DOWN:
            newHeadY = (newHeadY - 1 + NUM_Y_CELLS) % NUM_Y_CELLS;
            break;
        case LEFT:
            newHeadX = (newHeadX - 1 + NUM_X_CELLS) % NUM_X_CELLS;
            break;
        case RIGHT:
            newHeadX = (newHeadX + 1) % NUM_X_CELLS;
            break;
        default:
            return;  // No movement if joystick is neutral
    }
    
    // Check for collision with self
    for(int i = 0; i < snakeLength; i++) {
        if(snake[i].x == newHeadX && snake[i].y == newHeadY) {
            gameOver = true;
            return;
        }
    }
    
    // Check if snack was eaten
    bool snackEaten = (gameboard[newHeadX][newHeadY] == 1);
    
    // Move snake body
    if(!snackEaten) {
        // Clear last segment on gameboard
        gameboard[snake[snakeLength-1].x][snake[snakeLength-1].y] = 0;
        
        // Move body segments
        for(int i = snakeLength-1; i > 0; i--) {
            snake[i] = snake[i-1];
        }
    } else {
        // Increase snake length
        if(snakeLength < MAX_SNAKE_LENGTH) {
            snakeLength++;
            
            // Increase speed
            if(snakeSpeed > MIN_SNAKE_SPEED) {
                snakeSpeed -= SPEED_INCREASE;
                TIM3->ARR = snakeSpeed;  // Update timer period
            }
            
            // Generate new snack
            generateSnack();
        }
    }
    
    // Update head position
    snake[0].x = newHeadX;
    snake[0].y = newHeadY;
    
    // Update gameboard
    gameboard[newHeadX][newHeadY] = 2;  // Head
    for(int i = 1; i < snakeLength; i++) {
        gameboard[snake[i].x][snake[i].y] = 3;  // Body
    }
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
    if(TIM3->SR & TIM_SR_UIF) {  // If update interrupt flag is set
        TIM3->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        movementLogic();          // Call movement logic
    }
}