#include "main.h"
#include <string.h>


// Prototypes
void DMA_Init();
void USART_Init();
void DMA_to_USART(const char*);

int main() {

    USART_Init();
    DMA_Init();

    for(volatile int i = 0; i < 60000; i++);

    DMA_to_USART("=== Starting Program ===\r\n");  // Clear marker
    while(DMA1_Stream6->CR & (1 << 0));            // Wait for complete

    DMA_to_USART("Attempt #15\r\n");
    while(DMA1_Stream6->CR & (1 << 0));            // Wait for complete

    DMA_to_USART("Message 1\r\n");
    while(DMA1_Stream6->CR & (1 << 0));            // Wait for complete

    DMA_to_USART("Message 2\r\n");
    while(DMA1_Stream6->CR & (1 << 0));            // Wait for complete

    while(1) {
    }
}

// Initialize DMA
void DMA_Init() {
	RCC -> AHB1ENR |= (1 << 21);

	DMA1_Stream6 -> CR &= ~(1 << 0);
	while(DMA1_Stream6 -> CR & (1 << 0));

	DMA1_Stream6 -> CR &= ~(7 << 25);
	DMA1_Stream6 -> CR |= (4 << 25);

	DMA1_Stream6 -> CR |= (1 << 6);

	DMA1_Stream6 -> CR |= (1 << 10);
}

// Initialize USART for basic debugging
void USART_Init() {
	RCC -> APB1ENR |= (1 << 17);
	RCC -> AHB1ENR |= (1 << 0);

	GPIOA -> MODER &= ~((3 << (2 * 3)) | (3 << (3 * 3)));
	GPIOA -> MODER |= (2 << (2 * 2)) | (2 << (2 * 3));

	GPIOA -> AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3)));
	GPIOA -> AFR[0] |= (7 << (4 * 2)) | (7 << (4 * 3));

	USART2 -> BRR = 0x0683;
	USART2 -> CR1 |= (1 << 3);
	USART2 -> CR3 |= (1 << 7);

	volatile uint8_t temp = USART2->DR;
	temp = USART2->SR;
	(void)temp;

	USART2 -> CR1 |= (1 << 13);
}

// Initialize ADC

// Initialize I2C for LCD

// Initialize LCD

// DMA buffer print to LCD
void DMA_to_USART(const char* str) {
    while(DMA1_Stream6->CR & (1 << 0));

    if(DMA1->HISR & (1 << 21)) {
        DMA1->HIFCR |= (1 << 21);
    }

	uint16_t len = strlen(str);

	DMA1_Stream6 -> NDTR = len;
	DMA1_Stream6 -> PAR = (uint32_t)&USART2 -> DR;
	DMA1_Stream6 -> M0AR = (uint32_t)str;

	DMA1_Stream6 -> CR |= (1 << 0);

	while(DMA1_Stream6->CR & (1 << 0));
}
