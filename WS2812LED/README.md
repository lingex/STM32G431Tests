## Driver for WS2812LED using spi dma

## TIPS:
the WS2812 required baudrate @6.4Mbps, in this project, the clock is set to 102MHz and spi prescaler is set to 16 so we'll get a 6.375MHz spi baudrate, close to what the WS2812 required.

the 'DI' pin of the WS2812 should connect to the MOSI of the MCU, ignore the SCK.

![image](https://github.com/lingex/STM32G431Tests/blob/master/WS2812LED/PINOUT.jpg)

# Thanks to:

https://github.com/JassyL/STM32_WS2812B_HAL.git
