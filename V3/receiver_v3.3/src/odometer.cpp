// #include "odometer.hpp"
// #include <Arduino.h>
// #include "motion_control.hpp"


// #define PULSES_PER_REV 90.0 //change this
// #define REV_PER_PULSE (1.0 / PULSES_PER_REV)
// #define WHEEL_DIAMETER 8 //inches
// #define INCH_TO_KM 0.0000254
// #define MICROSEC_TO_HOUR 0.00000000027778

// const double TICK_TO_KM = REV_PER_PULSE * WHEEL_DIAMETER * 3.141592 * INCH_TO_KM;
// volatile long tickCount = 0;

// void odometerTask(void *){
//     //add alltime odometer reading here optionally
//     long prevTickCount = tickCount; 
//     unsigned long lastTime = micros();
//     while(true){
//         // double RPM;
//         double newVel;

//         unsigned long curTime = micros();
//         unsigned long deltaTime = curTime - lastTime;
//         long deltaTick = tickCount - prevTickCount;

//         if(deltaTime != 0){
//             // RPM = ((tickCount - prevTickCount) / PULSES_PER_REV) / ((lastTime - deltaTime) * MICROSEC_TO_MIN); //revolutions per hour

//             // Serial.println("delta km: " + String(TICK_TO_KM * deltaTick));
            
//             newVel = (TICK_TO_KM * deltaTick / (deltaTime * MICROSEC_TO_HOUR)); //km / hr
//         }
//         else newVel = 0;


//         // Serial.print("tickcount: " + String(tickCount));
//         // Serial.print(" prevTickCount: " + String(prevTickCount));
//         // Serial.println("tick to km: " + String(TICK_TO_KM  * 1000000.0));
//         // Serial.println(" delta tick: " + String(deltaTick));
//         // Serial.println("delta T: " + String(deltaTime));
//         // Serial.println("newVel: " + String(newVel, 10));

//         if(speedReadingMutex != NULL && xSemaphoreTake(speedReadingMutex, portMAX_DELAY) == pdTRUE){
//             // Serial.println("speedReading: " + String(speedReading, 10)); //THIS IS SITLL GIVING 0 EVEN THO NEW VEL ISN'T 0
//             // Serial.println("newVel: " + String(newVel, 10));
//             // speedReading = speedReading * 0.99992 + newVel * 0.00008; //tune this softness
//             speedReading = speedReading * 0.95 + newVel * 0.05;
//             // Serial.println("speedReading updated: " + String(speedReading, 10));
//             // Serial.println(String(speedReading, 10));
//             xSemaphoreGive(speedReadingMutex);
//         }

//         prevTickCount = tickCount;
//         lastTime = curTime;

//         // yield();
//         // vTaskDelay(pdMS_TO_TICKS(300));
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
//     vTaskDelete( NULL );
// }