
#ifndef __HW_SETUP_H__
#define __HW_SETUP_H__

/**
 * Defines Rosmo ESC's hardware setup (pins and peripherals).
 */


#include <SPI.h>
#include <Wire.h>


// pin assignments
#define STATUS_LED_PIN      PA10
#define WAKE_PIN            PA0
#define I2CADDR1_PIN        PD2
#define I2CADDR2_PIN        PD1

#define M0_INUH_PIN         PE9
#define M0_INUL_PIN         PE8
#define M0_INVH_PIN         PE11
#define M0_INVL_PIN         PE10
#define M0_INWH_PIN         PE13
#define M0_INWL_PIN         PE12

#define M0_AOUTU_PIN        PC1
#define M0_AOUTV_PIN        PC2
#define M0_AOUTW_PIN        PC3

#define M0_nFAULT_PIN       PC4
#define M0_nSLEEP_PIN       PC5

#define M1_INUH_PIN         PC6
#define M1_INUL_PIN         PB3
#define M1_INVH_PIN         PC7
#define M1_INVL_PIN         PB4
#define M1_INWH_PIN         PC9
#define M1_INWL_PIN         PD0

#define M1_AOUTU_PIN        PA1
#define M1_AOUTV_PIN        PA2
#define M1_AOUTW_PIN        PA3

#define M1_nFAULT_PIN       PD8
#define M1_nSLEEP_PIN       PD9

// voltage measurement
#define VBAT_PIN            PB2
#define VBAT_GAIN           ((2.2f+47.0f)/2.2f)
#define CURRENT_VpA         (2200.0f/9200.0f)

// spi bus 1 - Sensor 0
#define SENSOR0_nCS_PIN     PA4
#define CIPO0_PIN           PA6
#define COPI0_PIN           PA7
#define SCLK0_PIN           PA5
extern SPIClass SPI_Sensor0;

// spi bus 2 - Sensor 1
#define SENSOR1_nCS_PIN     PB12
#define CIPO1_PIN           PB14
#define COPI1_PIN           PB15
#define SCLK1_PIN           PB13
extern SPIClass SPI_Sensor1;

// spi bus 3 - M-BUS header
#define MBUS_nCS_PIN        PA15
#define CIPO2_PIN           PC11
#define COPI2_PIN           PC12
#define SCLK2_PIN           PC10
extern SPIClass SPI_MBus;

// i2c bus 1 - M-BUS header
#define I2C1_SCL_PIN        PA9
#define I2C1_SDA_PIN        PA8
// i2c bus 2 - Grove port
#define I2C2_SCL_PIN        PC8
#define I2C2_SDA_PIN        PB5



#endif
