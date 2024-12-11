# ADC Visualization with LCD Display
Real-time visualization of potentiometer readings using DMA for efficient data transfer, displaying both percentage and bar graph representation on an LCD screen.

## Hardware Requirements
- STM32F446RE
- 16x2 I2C LCD Display
- 10kΩ Potentiometer
- Jumper wires

## Pin Connections
### LCD (I2C1)
- VCC → 5V
- GND → GND
- SCL → PB8
- SDA → PB9

### Potentiometer
- VCC → 3.3V
- GND → GND
- WIPER → PA0 (ADC1_IN0)

## Software Components
- DMA for efficient ADC sampling
- I2C LCD communication
- Real-time display updates
- UART debugging interface

### Implementation Details
- Continuous ADC sampling using DMA
- Circular buffer for ADC readings
- 4-bit mode LCD operation
- Non-blocking operation
- Visual bar graph

## Features
- Real-time ADC value display (0-100%)
- Visual bar graph representation
- DMA-based data transfer and acquisition
- Configurable update rate
- Average value calculation

## Display Format
```
ADC: XX%
||||||||        
```
- Top line shows current percentage
- Bottom line shows proportional bar graph
- Updates continuously with potentiometer movement

## Usage
1. Connect hardware according to pin configurations
2. Build and flash program to STM32
3. Turn potentiometer to see real-time updates

## Performance Notes
- 100-sample moving average for stable readings
- DMA ensures no missed samples
- Efficient display updates
- Hardware-based timing
- Proper initialization sequences

![Output low](/images/Output_Low.png)
![Output high](/images/Output_High.png)