#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// Definição dos pinos
#define PIN_UMIDADE_SOLO 34
#define PIN_RELE 27
#define PIN_TRIG 26
#define PIN_ECHO 25
#define PIN_CHUVA 5

// Definição das filas
xQueueHandle fila_umidade_solo;

// Handles das tasks
TaskHandle_t handleSoloTask = NULL;
TaskHandle_t handleIrrigacaoTask = NULL;

// Task para realizar a leitura do sensor de umidade do solo
void vTaskSolo(void *pvParameters) {
  while (1) {
    int leituraADC = analogRead(PIN_UMIDADE_SOLO);
    long resposta = xQueueSend(fila_umidade_solo, &leituraADC, portMAX_DELAY);
    
    if (resposta) {
      ESP_LOGI("Leitura", "umidade de solo adicionado à fila");
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
void vTaskIrrigacao(void *pvParameters) {
  int solo_medicao;

  while(1) {
    if (xQueueReceive(fila_umidade_solo, &solo_medicao, portMAX_DELAY)) {
      Serial.printf("Valor bruto do sensor de umidade do solo: %d \n", solo_medicao);
    } else {
      ESP_LOGE("Medição", "dados não disponíveis");
    }
  }
};

// Task para enviar os dados por wi-fi
void vTaskComunicacao(void *pvParameters);

void setup() {
  Serial.begin(115200); // configura o serial
  pinMode(PIN_UMIDADE_SOLO, INPUT); // configura o pino para o sensor de umidade do solo
  // cria as filas
  fila_umidade_solo = xQueueCreate(5, sizeof(int));
  // cria as tasks
  xTaskCreate(vTaskSolo, "TASK_SOLO", 2048, NULL, 1, &handleSoloTask);
  xTaskCreate(vTaskIrrigacao, "TASK_IRRIGACAO", 2048, NULL, 1, &handleIrrigacaoTask);
}

void loop() {
  // nada aqui
}