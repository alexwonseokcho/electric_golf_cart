#ifndef _PINS_HPP
#define _PINS_HPP


#define LED_BUILTIN 8

//odrive left connected to motor ports
#define ODR_RX_LEFT 9
#define ODR_TX_LEFT 8

//odrive right connected to 
#define ODR_RX_RIGHT 4 //middle of the hall pins
#define ODR_TX_RIGHT 3 //bottom of the hall pins (make sure to short the right resistor pads, then connect the top pin to ground )

// IMU I2C
#define INT_PIN -1
#define SDA_PIN 1
#define SCL_PIN 0

// MCU I2C -- change these
#define ESP_I2C_SDA 5
#define ESP_I2C_SCL 6



// //odrive left connected to motor ports
// #define ODR_RX_LEFT 3
// #define ODR_TX_LEFT 2

// //odrive right connected to 
// #define ODR_RX_RIGHT 13 //middle of the hall pins
// #define ODR_TX_RIGHT 12 //bottom of the hall pins (make sure to short the right resistor pads, then connect the top pin to ground )

// // IMU I2C
// #define INT_PIN -1
// #define SDA_PIN 10
// #define SCL_PIN 9

// // MCU I2C -- change these
// #define ESP_I2C_SDA 5
// #define ESP_I2C_SCL 6


//motor specifications
#define PULSES_PER_REV 90.0
#define REV_PER_PULSE (1.0 / PULSES_PER_REV)
#define WHEEL_DIAMETER 8.5 //inches

//Acceleration profile [turns/s^2]
#define MAX_ACC 5.0 
#define MIN_ACC 0.5
#define MAX_PITCH_FOR_MIN_ACC 20.0 //degrees at which min acceleration is reached, past which it remains constant
// acc is calculated as a linear function of pitch, with a max value of MAX_ACC and a min value of MIN_ACC

#endif