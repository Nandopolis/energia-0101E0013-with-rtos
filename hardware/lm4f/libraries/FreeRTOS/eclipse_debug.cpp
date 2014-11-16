/*
 * eclipse_debug.cpp
 *
 *  Created on: 16/11/2014
 *      Author: murix
 */

#include <Energia.h>

// RTOS framework
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/osi.h>

void vTaskRedLed( void *pvParameters ){
   pinMode(RED_LED,OUTPUT);
   uint8_t pwm=0;
   while(1){
     pwm+=1;
     analogWrite(RED_LED,pwm);
     delay(10);
   }
}
void vTaskGreenLed( void *pvParameters ){
   pinMode(GREEN_LED,OUTPUT);
   uint8_t pwm=0;
   while(1){
     pwm+=2;
     analogWrite(GREEN_LED,pwm);
     delay(10);
   }
}
void vTaskBlueLed( void *pvParameters ){
   pinMode(BLUE_LED,OUTPUT);
   uint8_t pwm=0;
   while(1){
     pwm+=3;
     analogWrite(BLUE_LED,pwm);
     delay(10);
   }
}


void setup()
{
    //create new tasks to working in leds independently
    osi_TaskCreate( vTaskRedLed, ( signed portCHAR * ) "RED", OSI_DEFAULT_STACK_SIZE, NULL, 1, NULL );
    osi_TaskCreate( vTaskGreenLed, ( signed portCHAR * ) "GREEN", OSI_DEFAULT_STACK_SIZE,NULL, 1, NULL );
    osi_TaskCreate( vTaskBlueLed, ( signed portCHAR * ) "BLUE", OSI_DEFAULT_STACK_SIZE,NULL, 1, NULL );
}

void loop()
{
     //do nothing in default task
}


