#ifndef _CONFIG_H
#define _CONFIG_H


#include "Configuration_prusa.h"
#include "pins.h"

//ADC Configuration
#define ADC_CHAN_MSK      0b0000000010000111 //Used AD channels bit mask (0,1,2,7)
#define ADC_CHAN_CNT      4
#define ADC_OVRSAMPL      16        //oversampling multiplier
#define ADC_CALLBACK      adc_ready //callback function ()

//SWI2C configuration
#define SWI2C
//#define SWI2C_SDA         20 //SDA on P3
//#define SWI2C_SCL         21 //SCL on P3
#define SWI2C_A8
#define SWI2C_DEL         20 //2us clock delay
#define SWI2C_TMO         2048 //2048 cycles timeout

//SM4 configuration
#define SM4_DEFDELAY      500       //default step delay [us]


//W25X20CL configuration
//pinout:
#define W25X20CL_PIN_CS        32
//spi:
#define W25X20CL_SPI_RATE      0 // fosc/4 = 4MHz
#define W25X20CL_SPCR          SPI_SPCR(W25X20CL_SPI_RATE, 1, 1, 1, 0)
#define W25X20CL_SPSR          SPI_SPSR(W25X20CL_SPI_RATE)

//LANG - Multi-language support
//#define LANG_MODE              0 // primary language only
#define LANG_MODE              0 // sec. language support

#define LANG_SIZE_RESERVED     0x3000 // reserved space for secondary language (12288 bytes)


#endif //_CONFIG_H
