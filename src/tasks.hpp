#ifndef TASKS_H
#define TASKS_H
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <AccelStepper.h>

// Definição dos pinos
#define PIN_UMIDADE_SOLO 34
#define PIN_TRIG 5 // pino do sensor ultrassônico
#define PIN_ECHO 18 // pino do sensor ultrassônico
#define PIN_CHUVA 27
#define PIN_IN1 16  //pinos de configuração motor de passo
#define PIN_IN2 4 //pinos de configuração motor de passo
#define PIN_IN3 2 //pinos de configuração motor de passo
#define PIN_IN4 15 //pinos de configuração motor de passo
#define PIN_RELE 17

// Definição das filas
extern xQueueHandle fila_umidade_solo;
extern xQueueHandle fila_chuva;
extern xQueueHandle fila_nivel_agua;

// Handles das tasks
extern TaskHandle_t handleSoloTask;
extern TaskHandle_t handleIrrigacaoTask;
extern TaskHandle_t handleChuvaTask;
extern TaskHandle_t handleNivelAgua;

// Task para realizar a leitura do sensor de umidade do solo
void vTaskSolo(void *pvParameters);
// Task para realizar a leitura do sensor de chuva
void vTaskChuva(void *pvParameters);
// Task para realizar a leitura do sensor ultrassônico
void vTaskNivelAgua(void *pvParameters);
// Task para realizar a irrigação
void vTaskIrrigacao(void *pvParameters);
// Task para enviar os dados por wi-fi
void vTaskComunicacao(void *pvParameters);

#endif