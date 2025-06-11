#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Definição dos pinos
#define PIN_UMIDADE_SOLO 34
#define PIN_RELE 27
#define PIN_TRIG 26
#define PIN_ECHO 25
#define PIN_CHUVA 5

// Definição das filas
xQueueHandle fila_umidade_solo;

// Task para realizar a leitura do sensor de umidade do solo
void vTaskSolo(void *pvParameters) {
  while (1) {
    int leituraADC = analogRead(PIN_UMIDADE_SOLO);
    long resposta = xQueueSend(fila_umidade_solo, &leituraADC, portMAX_DELAY);
    
    if (resposta) {
      ESP_LOGI("Leitura", "Umidade de solo adicionado à fila");
    } else {
      ESP_LOGE("Leitura", "Falha ao enviar o dado de umidade de solo à fila");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task para realizar a leitura do sensor de chuva
void vTaskChuva(void *pvParameters);
// Task para realizar a leitura do sensor ultrassônico
void vTaskNivelAgua(void *pvParameters);
// Task para realizar a irrigação. Implementar a lógica
void vTaskIrrigacao(void *pvParameters);
// Task para enviar os dados por wi-fi
void vTaskComunicacao(void *pvParameters);

void setup() {
  fila_umidade_solo = xQueueCreate(5, sizeof(float));
}

void loop() {
  // nada aqui
}