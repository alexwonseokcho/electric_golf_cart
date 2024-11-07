#ifndef _PINS_HPP
#define _PINS_HPP


// #define LED_BUILTIN 48

#define steerServoPin 4


//odrive
#define ODR_RX 12
#define ODR_TX 11

// IMU I2C
#define INT_PIN 1
#define SDA_PIN 2
#define SCL_PIN 3

// MCU I2C -- change these
#define ESP_I2C_SDA 21
#define ESP_I2C_SCL 22


//motor specifications
#define PULSES_PER_REV 90.0
#define REV_PER_PULSE (1.0 / PULSES_PER_REV)
#define WHEEL_DIAMETER 8 //inches

#define ACC_LIMIT 5.0 //turns/s^2

#endif