#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// LCD
#define LCD_ADDR  				0x27				// LCD Address
#define LCD_EN 	  				0x04				// Enable
#define LCD_RW    				0x02				// Read/Write
#define LCD_RS    				0x01				// Register Select
#define LCD_BL	  				0x08				// Back-light

// LCD Commands
#define LCD_CLEAR 				0x01				// Clear Screen
#define LCD_HOME  				0x02				// Cursor Home
#define LCD_ENTRY_MODE  		0x06				// Increments cursor, no shift
#define LCD_DISPLAY_ON  		0x0C				// Display on, cursor off, blink off
#define LCD_FUNCTION_SET 		0x28				// 4-bit mode, 2 lines, 5x8 font

// Prototypes
void USART_DMA_Init();
void USART_Init();
void DMA_to_USART(const char*);
void ADC_Init();
void ADC_DMA_Init();

void I2C_Init();
void I2C_Write(uint8_t, uint8_t);
void LCD_SendCommand(uint8_t);
void LCD_Init();
void LCD_SendData(uint8_t);
void LCD_SendString(char*);
void LCD_Update();

#define MAX_BUFFER 100
char print_buffer[50];
uint16_t adc_buffer[MAX_BUFFER];

int main() {
    USART_Init();
    USART_DMA_Init();
    I2C_Init();
    LCD_Init();
    ADC_Init();
    ADC_DMA_Init();

    for (volatile int i = 0; i < 70000; i++);

    DMA2_Stream4->CR |= (1 << 0);
    ADC1->CR2 |= (1 << 0);
    ADC1->CR2 |= (1 << 30);

    while(1) {
        LCD_Update();
        for(volatile int i = 0; i < 750000; i++);
    }
}

// Initialize DMA for USART
void USART_DMA_Init() {
	RCC -> AHB1ENR |= (1 << 21);										// DMA1 Clock

	DMA1_Stream6 -> CR &= ~(1 << 0);									// Disables the stream and waits until ready
	while(DMA1_Stream6 -> CR & (1 << 0));

	DMA1_Stream6 -> CR &= ~(7 << 25);									// Clears and sets the channel to 4
	DMA1_Stream6 -> CR |= (4 << 25);

	DMA1_Stream6 -> CR |= (1 << 6);										// Sets direction to peripheral from memory

	DMA1_Stream6 -> CR |= (1 << 10);									// Sets memory increment mode
}

// Initialize USART for basic debugging
void USART_Init() {
	RCC -> APB1ENR |= (1 << 17);										// USART2 Clock
	RCC -> AHB1ENR |= (1 << 0);											// GPIOA Clock

	GPIOA -> MODER &= ~((3 << (2 * 3)));								// Sets RX to alternate function
	GPIOA -> MODER |= (2 << (2 * 2));

	GPIOA -> AFR[0] &= ~((0xF << (4 * 2)));								// Sets alternate function to USART2
	GPIOA -> AFR[0] |= (7 << (4 * 2));

	USART2 -> BRR = 0x0683;												// 9600 Baud Rate
	USART2 -> CR1 |= (1 << 3);											// Transmitter Enable
	USART2 -> CR3 |= (1 << 7);											// DMA Enable

	USART2 -> CR1 |= (1 << 13);											// USART2 Enable
}

// Initialize ADC
void ADC_Init() {
	RCC -> APB2ENR |= (1 << 8);											// ADC Clock
	RCC -> AHB1ENR |= (1 << 0);											// GPIOA Clock

	GPIOA -> MODER &= ~(3 << 0);										// Set to analog mode
	GPIOA -> MODER |= (3 << 0);

	ADC1 -> CR2 |= (1 << 8);											// DMA Enabled
	ADC1 -> CR2 |= (1 << 9);											// DMA Continuous
	ADC1 -> CR2 |= (1 << 1);											// Continuous Conversion
	ADC1 -> SMPR2 |= (4 << 0);											// 84 Cycles; Sampling Time

	ADC1 -> SQR1 &= ~(0xF << 20);     									// 1 conversion
	ADC1 -> SQR3 &= ~(0x1F << 0);     									// Clear channel selection
	ADC1 -> SQR3 |= (0 << 0);         									// Use channel 0
}

// Initialize DMA for ADC
void ADC_DMA_Init() {
	RCC -> AHB1ENR |= (1 << 22);										// DMA2 Clock

	DMA2_Stream4 -> CR &= ~(1 << 0);									// Disables the stream and waits until ready
	while(DMA2_Stream4 -> CR & (1 << 0));

	DMA2_Stream4 -> CR &= ~(7 << 25);									// Clears and sets the channel to 0

	DMA2_Stream4 -> CR |= (1 << 10);									// Sets memory increment mode
	DMA2_Stream4 -> CR |= (1 << 8);										// Sets circular buffer mode
	DMA2_Stream4 -> CR |= (1 << 13);									// Memory size 16-bit
	DMA2_Stream4 -> CR |= (1 << 11);									// Peripheral size 16-bit

	DMA2_Stream4 -> NDTR = MAX_BUFFER;									// Number of data items
	DMA2_Stream4 -> PAR = (uint32_t)&ADC1 -> DR;						// Pulls from ADC1 data register
	DMA2_Stream4 -> M0AR = (uint32_t)&adc_buffer;						// Outputs to ADC buffer incrementally
}

// DMA buffer print to USART; Used for debugging
void DMA_to_USART(const char* str) {
    while(DMA1_Stream6->CR & (1 << 0));									// Waits for stream to disable

    if(DMA1->HISR & (1 << 21)) {										// Checks if the transfer is complete; Extra protection
        DMA1->HIFCR |= (1 << 21);
    }

	uint16_t len = strlen(str);

	DMA1_Stream6 -> NDTR = len;											// Sets number of data values to length of string
	DMA1_Stream6 -> PAR = (uint32_t)&USART2 -> DR;						// Sets peripheral output address to USART2's data register
	DMA1_Stream6 -> M0AR = (uint32_t)str;								// Sets input data to the string

	DMA1_Stream6 -> CR |= (1 << 0);										// Enables the stream
}

// Initialize I2C for LCD
void I2C_Init() {
	RCC -> AHB1ENR |= (1 << 1);											// Enable GPIOB Clock

	GPIOB -> MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2))); 				// Clears mode bits for PB8 and PB9
	GPIOB -> MODER |= (2 << (8 * 2)) | (2 << (9 *2));					// Sets mode to Alternate Function

	GPIOB -> OTYPER |= (1 << 8) | (1 << 9);								// Turns on Open-Drain for PB8 and PB9

	GPIOB -> PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));				// Clears Pull-Up/Pull-Down Register
	GPIOB -> PUPDR |= (1 << (8 * 2)) | (1 << (9 * 2));					// Sets pins to Pull-Up

	GPIOB -> AFR[1] &= ~(0xF << 0) | (0xF << 4);						// Clears Alternate Function for PB8 and PB9
	GPIOB -> AFR[1] |= (4 << 0) | (4 << 4);								// Sets the Alternate Function to AF4 for PB8 and PB9

	RCC -> APB1ENR |= (1 << 21); 										// Enable I2C Clock

	I2C1 -> CR1 |= (1 << 15);											// Toggles SWRST; Resets the I2C connection
	I2C1 -> CR1 &= ~(1 << 15);											// Disables SWRST; I2C is ready

	I2C1 -> CR2 |= (42 << 0);											// Sets clock frequency to 42MHz

	I2C1 -> CCR = 210;													// Configured for 100kHz; Standard frequency

	I2C1 -> TRISE = 43;													// Frequency + 1; Used to set maximum rise time

	I2C1 -> CR1 |= (1 << 0);											// Enables I2C
}

// Checks if I2C bus is free
uint8_t I2C_CheckBusy() {
	if (I2C1 -> SR1 & (1 << 1)) {
		return 1;
	}
	return 0;
}

// Generates Start condition
void I2C_Start() {
	I2C1 -> CR1 |= (1 << 8);

	while (!(I2C1 -> SR1 & (1 << 0)));
}

// Sends address of target device
void I2C_SendAddress(uint8_t address, uint8_t read) {
	I2C1 -> DR = (address << 1) | read;

	while (!(I2C1 -> SR1 & (1 << 1)));

	uint8_t dummyRead = I2C1 -> SR1;
	dummyRead = I2C1 -> SR2;
	(void)dummyRead;
}

// Sends a byte of data and waits
void I2C_SendData(uint8_t data) {
	while (!(I2C1 -> SR1 & (1 << 7)));

	I2C1 -> DR = data;

	while (!(I2C1 -> SR1 & (1 << 2)));
}

// Generates stop condition
void I2C_Stop() {
	I2C1 -> CR1 |= (1 << 9);
}

// Sends a byte of data to a target device
void I2C_Write(uint8_t address, uint8_t data) {
	I2C_CheckBusy();
	I2C_Start();
	I2C_SendAddress(address, 0);
	I2C_SendData(data);
	I2C_Stop();
}

// Sends a command to the LCD for control
void LCD_SendCommand(uint8_t cmd) {
	uint8_t upper = (cmd & 0xF0) | LCD_BL;
	uint8_t lower = ((cmd << 4) & 0xF0) | LCD_BL;

	I2C_Write(LCD_ADDR,	upper | LCD_EN);
	I2C_Write(LCD_ADDR, upper);

	I2C_Write(LCD_ADDR, lower | LCD_EN);
	I2C_Write(LCD_ADDR, lower);
}

// Initialize LCD
void LCD_Init() {
	for (volatile int i = 0; i < 50000; i++);

	LCD_SendCommand(0x33);												// Initializes LCD
	LCD_SendCommand(0x32);												// Sets function to 4-bit mode
	LCD_SendCommand(LCD_FUNCTION_SET);									// Configures 4-bit mode
	LCD_SendCommand(LCD_DISPLAY_ON);									// Turns on display
	LCD_SendCommand(LCD_ENTRY_MODE);									// Sets entry mode
	LCD_SendCommand(LCD_CLEAR);											// Clears screen
}

// Writes a byte of data to the LCD
void LCD_SendData(uint8_t data) {
	uint8_t upper = (data & 0xF0) | LCD_BL | LCD_RS;
	uint8_t lower = ((data << 4) & 0xF0) | LCD_BL | LCD_RS;

	I2C_Write(LCD_ADDR, upper | LCD_EN);
	I2C_Write(LCD_ADDR, upper);

	I2C_Write(LCD_ADDR, lower | LCD_EN);
	I2C_Write(LCD_ADDR, lower);
}

// Users a character pointer to print a string
void LCD_SendString(char *str) {
    while(*str != '\0') {
        LCD_SendData(*str);
        str++;
    }
}

// Prints to LCD screen with a visual based on adc_buffer
// Calculates the percentage of adc_buffer and calculates the amount of bars to prints
// Example output

// ADC: 95%
// | | | | | | | | | | | | | |
void LCD_Update() {
    uint32_t sum = 0;
    for(int i = 0; i < MAX_BUFFER; i++) {
        sum += adc_buffer[i];
    }

    float average = (float)sum / MAX_BUFFER;
    uint8_t percent = (uint8_t)ceil(((average / 4095.0) * 100));
    uint8_t num_bars = (percent * 16 + 99) / 100;

    char lcd_buffer[17];

    LCD_SendCommand(LCD_CLEAR);
    for(volatile int i = 0; i < 10000; i++);

    LCD_SendCommand(0x80 | 0x00);

    sprintf(lcd_buffer, "ADC: %d%%", percent);
    LCD_SendString(lcd_buffer);

    LCD_SendCommand(0x80 | 0x40);

    for(int i = 0; i < 16; i++) {
        if(i < num_bars) {
            LCD_SendData('|');
        } else {
            LCD_SendData(' ');
        }
    }
}
