#include "main.h"
#include <string.h>


// Prototypes
void DMA_Init();
void USART_Init();
void DMA_to_USART(const char*);

int main() {
    USART_Init();
    DMA_Init();

    for(volatile int i = 0; i < 65000; i++);

    DMA_to_USART("=== Starting Program ===\r\n");
    DMA_to_USART("Attempt #26\r\n");
    DMA_to_USART("Message 1\r\n");
    DMA_to_USART("Message 2\r\n");

    while(1) {
    }
}

// Initialize DMA
void DMA_Init() {
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

// Initialize I2C for LCD

// Initialize LCD

// DMA buffer print to LCD
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
