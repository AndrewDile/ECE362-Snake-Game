/**
  ******************************************************************************
  * @file    support.c
  * @author  Andrew Dile, Abhi Annigeri, Daniel Wang, William Lee
  * @date    November 4 2024
  * @brief   ECE 362 Snake Game
  ******************************************************************************
*/

#include "header.h"
#define IDLE 0
#define PLAYING 1
#define GAME_OVER 2
#define MAX_SNAKE_LENGTH (NUM_X_CELLS * NUM_Y_CELLS)
#define INITIAL_SNAKE_LENGTH 3
#define INITIAL_SNAKE_SPEED 500  // milliseconds between moves
#define MIN_SNAKE_SPEED 100      // maximum speed (minimum delay)
#define SPEED_INCREASE 25        // ms faster per snack eaten
#define NUM_X_CELLS 20 // Number of cells horizontally
#define NUM_Y_CELLS 20 // Number of cells vertically
#define NEUTRAL 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define PWM_MAX 2400




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

void init_gpio() {
    RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);
    GPIOA->MODER |= (GPIO_MODER_MODER0 | GPIO_MODER_MODER1); // Joystick PA0, PA1 as analog
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1); // LED PB0, PB1 clear mode
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0); // Set PB0, PB1 as output
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1); // High speed for LEDs
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1); // No pull-up or pull-down for LEDs
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


// int read_joystick_x() {

// }

// int read_joystick_y() {

// }

int8_t getJoystickDirection(void) {
    int x = read_joystick_x();
    int y = read_joystick_y();
    if (x > 200) return RIGHT;
    else if (x < 50) return LEFT;
    else if (y > 200) return UP;
    else if (y < 50) return DOWN;
    else return NEUTRAL;
}

void updateJoystickDirection(void) {
    joystickDirection = getJoystickDirection();
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
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 7999;
    TIM3->ARR = 999;
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CCR1 = 500;
    TIM3->CR1 |= TIM_CR1_CEN;
}

// plays song based on value, maybe updates a flag once it finishes a song?
void playSound(uint8_t song) {
    switch(song) {
        case 0:
            TIM3->ARR = 999;
            break;
        case 1:
            TIM3->ARR = 799;
            break;
        case 2:
            TIM3->ARR = 599;
            break;
        default:
            TIM3->CCR1 = 0; //muted 
            return;
    }
    TIM3->CCR1 = TIM3->ARR / 2; //maintains 50% cycle 

}

int getrgb(void); //helper function for setrgb

// Helper function for you
// Accept a byte in BCD format and convert it to decimal
uint8_t bcd2dec(uint8_t bcd) {
    // Lower digit
    uint8_t dec = bcd & 0xF;

    // Higher digit
    dec += 10 * (bcd >> 4);
    return dec;
}

// set LED color based on value, should be able to pull pretty much verbatum from PWM lab
void setrgb(int rgb) {
    uint8_t b = bcd2dec(rgb & 0xFF); //blue
    uint8_t g = bcd2dec((rgb >> 8) & 0xFF); //green
    uint8_t r = bcd2dec((rgb >> 16) & 0xFF); //red

    uint8_t red_duty = 100 - r;
    uint8_t green_duty = 100 - g;
    uint8_t blue_duty = 100 - b;

    uint16_t red_scaled = (red_duty * PWM_MAX) / 100;
    uint16_t green_scaled = (green_duty * PWM_MAX) / 100;
    uint16_t blue_scaled = (green_duty * PWM_MAX) / 100;

    TIM1->CCR1 = red_scaled;   // Red LED (connected to TIM1->CCR1)
    TIM1->CCR2 = green_scaled; // Green LED (connected to TIM1->CCR2)
    TIM1->CCR3 = blue_scaled;  // Blue LED (connected to TIM1->CCR3)
}




// set up game display (and maybe SD interface?) to be communicated with
void setupGameDisplay() {

}

// update the game display based on current gameboard values
void updateGameDisplay() {
    clearDisplay();
    for (int i = 0; i < snakeLength; i++) {
        drawSegment(snake[i].x, snake[i].y);
    }
    drawSnack();
}

// IRQ Handler that calls movementLogic() (TIM3 arbitrarily chosen, can be changed)
void TIM3_IRQHandler() {
    if(TIM3->SR & TIM_SR_UIF) {  // If update interrupt flag is set
        TIM3->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        movementLogic();          // Call movement logic
    }
}