#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// Definição dos pinos
#define PIN_UMIDADE_SOLO 34
#define PIN_RELE 8
#define PIN_TRIG 5 // pino do sensor ultrassônico
#define PIN_ECHO 18 // pino do sensor ultrassônico
#define PIN_CHUVA 27

// Definição das filas
xQueueHandle fila_umidade_solo;
xQueueHandle fila_chuva;
xQueueHandle fila_nivel_agua;

// Handles das tasks
TaskHandle_t handleSoloTask = NULL;
TaskHandle_t handleIrrigacaoTask = NULL;
TaskHandle_t handleChuvaTask = NULL;
TaskHandle_t handleNivelAgua = NULL;

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
void vTaskChuva(void *pvParameters) {
  while(1) {
    int leituraADC = analogRead(PIN_CHUVA);
    long resposta = xQueueSend(fila_chuva, &leituraADC, portMAX_DELAY);

    if (resposta) {
      ESP_LOGI("Leitura", "sensor chuva adicionado à fila");
    } else {
      ESP_LOGE("Leitura", "Falha ao enviar o dado do sensor de chuva à fila");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task para realizar a leitura do sensor ultrassônico
void vTaskNivelAgua(void *pvParameters) {
  long duracao;
  float distancia_cm;

  while(1) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // medição do tempo de retorno do pulso
    duracao = pulseIn(PIN_ECHO, HIGH);

    // calcula a distância em cm
    distancia_cm = duracao * 0.034 / 2;

    xQueueSend(fila_nivel_agua, &distancia_cm, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task para realizar a irrigação. Implementar a lógica para acionar o motor após chuva intensa e muita umidade do solo
void vTaskIrrigacao(void *pvParameters) {
  int solo_medicao;
  int chuva_medicao;
  float nivel_medicao;

  while(1) {
    if (xQueueReceive(fila_umidade_solo, &solo_medicao, portMAX_DELAY) &&
        xQueueReceive(fila_chuva, &chuva_medicao, portMAX_DELAY) &&
        xQueueReceive(fila_nivel_agua, &nivel_medicao, portMAX_DELAY)) {

      Serial.printf("Valor bruto do sensor de umidade do solo: %d \n", solo_medicao);
      Serial.printf("Valor bruto do sensor de chuva: %d \n", chuva_medicao);
      Serial.printf("Distância em cm: %f\n", nivel_medicao);

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
  pinMode(PIN_CHUVA, INPUT); // configura o pino para o sensor de chuva
  pinMode(PIN_ECHO, INPUT);//configura o pino echo do sensor ultrassonico
  pinMode(PIN_TRIG, OUTPUT);//configura o pino trig do sensor ultrassonico
  // cria as filas
  fila_umidade_solo = xQueueCreate(5, sizeof(int));
  fila_chuva = xQueueCreate(5, sizeof(int));
  fila_nivel_agua = xQueueCreate(5, sizeof(float));
  // cria as tasks
  xTaskCreate(vTaskSolo, "TASK_SOLO", 2048, NULL, 1, &handleSoloTask);
  xTaskCreate(vTaskChuva, "TASK_CHUVA", 2048, NULL, 1, &handleChuvaTask);
  xTaskCreate(vTaskIrrigacao, "TASK_IRRIGACAO", 3072, NULL, 1, &handleIrrigacaoTask);
  xTaskCreate(vTaskNivelAgua, "TASK_NIVEL_AGUA", 2048, NULL, 1, &handleNivelAgua);
}

void loop() {
  // nada aqui
}