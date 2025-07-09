#include "tasks.hpp"

void setup() {
  Serial.begin(115200); // configura o serial
  pinMode(PIN_UMIDADE_SOLO, INPUT); // configura o pino para o sensor de umidade do solo
  pinMode(PIN_CHUVA, INPUT); // configura o pino para o sensor de chuva
  pinMode(PIN_ECHO, INPUT);//configura o pino echo do sensor ultrassonico
  pinMode(PIN_TRIG, OUTPUT);//configura o pino trig do sensor ultrassonico
  pinMode(PIN_RELE, OUTPUT); //configura o pino do relé
  // cria as filas
  fila_umidade_solo = xQueueCreate(5, sizeof(int));
  fila_chuva = xQueueCreate(5, sizeof(int));
  fila_nivel_agua = xQueueCreate(5, sizeof(float));
  fila_dados_irrigacao = xQueueCreate(5, sizeof(DadosIrrigacao));
  // cria as tasks
  xTaskCreatePinnedToCore(vTaskSolo, "TASK_SOLO", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &handleSoloTask, 1);
  xTaskCreatePinnedToCore(vTaskChuva, "TASK_CHUVA", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &handleChuvaTask, 1);
  xTaskCreatePinnedToCore(vTaskIrrigacao, "TASK_IRRIGACAO", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &handleIrrigacaoTask, 0);
  xTaskCreatePinnedToCore(vTaskNivelAgua, "TASK_NIVEL_AGUA", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &handleNivelAgua, 1);
  xTaskCreatePinnedToCore(vTaskComunicacao, "TASK_WIFI", configMINIMAL_STACK_SIZE + 2028, NULL, 1, &handleComunicacao, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(500));
}