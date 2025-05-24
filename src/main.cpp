#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

void vTaskLeituraSensores(void *pvParameters);
void vTaskIrrigacao(void *pvParameters);
void vTaskComunicacao(void *pvParameters);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}