#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2023-01-06 20:24:00

#include "Arduino.h"
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

void setup(void) ;
void vTask_shiftlight(void *pvParameters) ;
void vTask_pedal(void *pvParameters) ;
void vTask_motor(void *pvParameters) ;
void Handler_hall(void) ;
void vTask_hall(void *pvParameters) ;
void Handler_down_shift(void) ;
void Handler_up_shift(void) ;
void vTask_down_shift(void *pvParameters) ;
void vTask_up_shift(void *pvParameters) ;
void vTask_display(void *pvParameters) ;
void retangulos(short x, short y) ;
void escreve_texto(char *text, uint16_t color, short x, short y, 		short font_size) ;
void loop() ;

#include "Projeto_final.ino"


#endif
