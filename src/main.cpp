#include "tasks.hpp"
#include "ultrassom_interrupcao.hpp"
#include "ConfigManager.hpp"
#include "PairingManager.hpp"

void setup() {
  Serial.begin(115200); // configura o serial

  setupConfig(); // inicializa o sistema de preferências
  
  Serial.println("Iniciando tentativa de conexao Wi-Fi...");
  setupAndAttemptConnection(deviceConfig);
  Serial.println("Conexao estabelecida. Iniciando tasks da aplicacao.");

  if (loadConfig(deviceConfig)) {
    Serial.println("Configuracao carregada da memoria:");
    Serial.println("SSID: " + deviceConfig.ssid);
    Serial.println("URL: " + deviceConfig.serverUrl);
    Serial.println("API Key: " + deviceConfig.apiKey);
  } else {
    Serial.println("Nenhuma configuracao encontrada na memoria. O dispositivo precisa ser configurado.");
  }

  pinMode(PIN_UMIDADE_SOLO, INPUT); // configura o pino para o sensor de umidade do solo
  pinMode(PIN_CHUVA, INPUT); // configura o pino para o sensor de chuva
  setupUltrassom(PIN_TRIG, PIN_ECHO); // configura os pinos do sensor ultrassonico
  pinMode(PIN_RELE, OUTPUT); //configura o pino do relé
  // cria as filas
  fila_umidade_solo = xQueueCreate(5, sizeof(int));
  fila_chuva = xQueueCreate(5, sizeof(int));
  fila_nivel_agua = xQueueCreate(5, sizeof(float));
  fila_dados_irrigacao = xQueueCreate(5, sizeof(DadosIrrigacao));
  // cria as tasks
  xTaskCreatePinnedToCore(vTaskSolo, "TASK_SOLO", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &handleSoloTask, 1);
  xTaskCreatePinnedToCore(vTaskMotor, "TASK_MOTOR", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vTaskChuva, "TASK_CHUVA", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &handleChuvaTask, 1);
  xTaskCreatePinnedToCore(vTaskIrrigacao, "TASK_IRRIGACAO", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &handleIrrigacaoTask, 0);
  xTaskCreatePinnedToCore(vTaskNivelAgua, "TASK_NIVEL_AGUA", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &handleNivelAgua, 1);
  xTaskCreatePinnedToCore(vTaskComunicacao, "TASK_WIFI", configMINIMAL_STACK_SIZE + 2028, NULL, 1, &handleComunicacao, 0);
  // configura o watchdog
  esp_task_wdt_init(5, true); // inicia o watchdog com 5 segundos de timeout
  esp_task_wdt_add(handleIrrigacaoTask);
  esp_task_wdt_add(handleComunicacao);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}