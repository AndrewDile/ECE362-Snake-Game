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

// void init_gpio() {
//     RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);
//     GPIOA->MODER |= (GPIO_MODER_MODER0 | GPIO_MODER_MODER1); // Joystick PA0, PA1 as analog
//     GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1); // LED PB0, PB1 clear mode
//     GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0); // Set PB0, PB1 as output
//     GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1); // High speed for LEDs
//     GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1); // No pull-up or pull-down for LEDs
// }

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
  static int8_t x = -1;
  static int8_t y = -1;

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

      snake[0].direction = joystickDirection;

      // start new ADC conversion
      ADC1->CR |= ADC_CR_ADSTART;
      x = -1;
      y = -1;
    }
  }
}

void initializeSnake() {
    // Start snake in middle of board
    // uint8_t startX = NUM_X_CELLS / 2;
    // uint8_t startY = NUM_Y_CELLS / 2;
    
    // // Initialize snake segments
    // for(int i = 0; i < snakeLength; i++) {
    //     snake[i].x = startX - i;  // Snake starts horizontally
    //     snake[i].y = startY;
    //     gameboard[startX - i][startY] = (i == 0) ? 2 : 3;  // 2 for head, 3 for body
    // }
    
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
    // array of available spaces
    // iterating over the whole gameboard
    // random index in that range

    // uint8_t x, y;
    // bool validPosition;
    
    // do {
    //     validPosition = true;
        
    //     // Generate random position
    //     x = rand() % NUM_X_CELLS;
    //     y = rand() % NUM_Y_CELLS;
        
    //     // Check if position is occupied by snake
    //     for(int i = 0; i < snakeLength; i++) {
    //         if(snake[i].x == x && snake[i].y == y) {
    //             validPosition = false;
    //             break;
    //         }
    //     }
    // } while(!validPosition);
    
    // Place snack on gameboard
    // gameboard[x][y] = 1;  // 1 represents snack
}

// iterates over snake and updates segments and gameboard, maybe calls playSound based on what is happening?
void movementLogic() {
// iterate over snake
// check for snack
// update snake head along with all segments
// update snake position based on direction
// update the game board

//    if(gameOver) return;
    
//     // Calculate new head position
//     uint8_t newHeadX = snake[0].x;
//     uint8_t newHeadY = snake[0].y;
    
//     // Update position based on joystick direction
//     switch(joystickDirection) {
//         case UP:
//             newHeadY = (newHeadY + 1) % NUM_Y_CELLS;
//             break;
//         case DOWN:
//             newHeadY = (newHeadY - 1 + NUM_Y_CELLS) % NUM_Y_CELLS;
//             break;
//         case LEFT:
//             newHeadX = (newHeadX - 1 + NUM_X_CELLS) % NUM_X_CELLS;
//             break;
//         case RIGHT:
//             newHeadX = (newHeadX + 1) % NUM_X_CELLS;
//             break;
//         default:
//             return;  // No movement if joystick is neutral
//     }
    
//     // Check for collision with self
//     for(int i = 0; i < snakeLength; i++) {
//         if(snake[i].x == newHeadX && snake[i].y == newHeadY) {
//             gameOver = true;
//             return;
//         }
//     }
    
//     // Check if snack was eaten
//     bool snackEaten = (gameboard[newHeadX][newHeadY] == 1);
    
//     // Move snake body
//     if(!snackEaten) {
//         // Clear last segment on gameboard
//         gameboard[snake[snakeLength-1].x][snake[snakeLength-1].y] = 0;
        
//         // Move body segments
//         for(int i = snakeLength-1; i > 0; i--) {
//             snake[i] = snake[i-1];
//         }
//     } else {
//         // Increase snake length
//         if(snakeLength < MAX_SNAKE_LENGTH) {
//             snakeLength++;
            
//             // Increase speed
//             if(snakeSpeed > MIN_SNAKE_SPEED) {
//                 snakeSpeed -= SPEED_INCREASE;
//                 TIM3->ARR = snakeSpeed;  // Update timer period
//             }
            
//             // Generate new snack
//             generateSnack();
//         }
//     }
    
//     // Update head position
//     snake[0].x = newHeadX;
//     snake[0].y = newHeadY;
    
//     // Update gameboard
//     gameboard[newHeadX][newHeadY] = 2;  // Head
//     for(int i = 1; i < snakeLength; i++) {
//         gameboard[snake[i].x][snake[i].y] = 3;  // Body
//     }
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
    uint16_t blue_scaled = (blue_duty * PWM_MAX) / 100;

    TIM1->CCR1 = red_scaled;   // Red LED (connected to TIM1->CCR1)
    TIM1->CCR2 = green_scaled; // Green LED (connected to TIM1->CCR2)
    TIM1->CCR3 = blue_scaled;  // Blue LED (connected to TIM1->CCR3)
}

// set up game display (and maybe SD interface?) to be communicated with
void setupGameDisplay() {

}

// update the game display based on current gameboard values
// void updateGameDisplay() {
//     clearDisplay();
//     for (int i = 0; i < snakeLength; i++) {
//         drawSegment(snake[i].x, snake[i].y);
//     }
//     drawSnack();
// }

// IRQ Handler that calls movementLogic() (TIM3 arbitrarily chosen, can be changed)
void TIM3_IRQHandler() {
    if(TIM3->SR & TIM_SR_UIF) {  // If update interrupt flag is set
        TIM3->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        movementLogic();          // Call movement logic
    }
}

void init_spi1_slow() {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);

    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
    GPIOB->AFR[0] |= (0x00 << GPIO_AFRL_AFSEL3_Pos) | (0x00 << GPIO_AFRL_AFSEL4_Pos) | (0x00 << GPIO_AFRL_AFSEL5_Pos);

    SPI1->CR1 = 0;
    SPI1->CR2 = 0;

    SPI1->CR1 |= (0b111 << SPI_CR1_BR_Pos);
    SPI1->CR2 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

    SPI2->CR2 |= (0b111 << SPI_CR2_DS_Pos);
    SPI2->CR2 |= SPI_CR2_FRXTH;

    SPI1->CR1 |= SPI_CR1_SPE;
}

void enable_sdcard() {
    GPIOB->BSRR |= (1 << 18);
}

void disable_sdcard() {
    GPIOB->BSRR |= (1 << 2);
}

void init_sdcard_io() {
    init_spi1_slow();
    GPIOB->MODER &= ~(GPIO_MODER_MODER2);
    GPIOB->MODER |= 0b01 << GPIO_MODER_MODER2_Pos;
    disable_sdcard();
}

void sdcard_io_high_speed() {
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= (0b001 << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
}

void init_lcd_spi() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0);
    init_spi1_slow();
    sdcard_io_high_speed();
}

void systick_init(void) {
    // Disable SysTick during initialization
    SysTick->CTRL = 0;
    
    // Set maximum reload value
    SysTick->LOAD = 0xFFFFFF;
    
    // Clear current value
    SysTick->VAL = 0;
    
    // Enable SysTick with processor clock
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}