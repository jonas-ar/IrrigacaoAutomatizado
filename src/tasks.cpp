#include "tasks.hpp"

// Definição das filas
QueueHandle_t fila_umidade_solo;
QueueHandle_t fila_chuva;
QueueHandle_t fila_nivel_agua;

// Handles das tasks
TaskHandle_t handleSoloTask;
TaskHandle_t handleChuvaTask;
TaskHandle_t handleIrrigacaoTask;
TaskHandle_t handleNivelAgua;

// Task para realizar a leitura do sensor de umidade do solo
void vTaskSolo(void *pvParameters) {
  while (1) {
    int leituraADC = analogRead(PIN_UMIDADE_SOLO);
    long resposta = xQueueSend(fila_umidade_solo, &leituraADC, portMAX_DELAY);
    
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task para realizar a leitura do sensor de chuva
void vTaskChuva(void *pvParameters) {
  while(1) {
    int leituraADC = digitalRead(PIN_CHUVA); // 0 chuva, 1 não chuva
    long resposta = xQueueSend(fila_chuva, &leituraADC, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(500));
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
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// instância do motor
AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);

// Task para realizar a irrigação. Implementar a lógica para acionar o motor após chuva intensa e muita umidade do solo
void vTaskIrrigacao(void *pvParameters) {
  int solo_medicao;
  int estado_chuva;
  float nivel_medicao;

  // configurações do motor de passo
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(200);
  stepper.setSpeed(50);
  bool lonaFechada = false;

  while(1) {
    if (xQueueReceive(fila_umidade_solo, &solo_medicao, portMAX_DELAY) &&
        xQueueReceive(fila_chuva, &estado_chuva, portMAX_DELAY) &&
        xQueueReceive(fila_nivel_agua, &nivel_medicao, portMAX_DELAY)) {

      Serial.printf("Valor bruto do sensor de umidade do solo: %d \n", solo_medicao);
      Serial.printf("Valor bruto do sensor de chuva (0 chovendo, 1 sem chover): %d \n", estado_chuva);
      Serial.printf("Distância em cm: %f\n\n", nivel_medicao);

      if(solo_medicao > 2500) {
        Serial.println("RELE LIGADO, BAIXA HUMIDADE DO SOLO");
        digitalWrite(PIN_RELE, HIGH);
      } else {
        Serial.println("RELE DESLIGADO, UMIDADE ESTA OK!");
        digitalWrite(PIN_RELE, LOW);
      }      

      // se estiver chovendo e o estado atual da lona for aberta, será fechado
      if (estado_chuva == 0 && !lonaFechada) {
        Serial.println("Está chovendo! acionando cobertura...");
        stepper.moveTo(stepper.currentPosition() + 2048); // fecha a cobertura

        while (stepper.distanceToGo() != 0) {
          stepper.run();
          vTaskDelay(pdMS_TO_TICKS(1)); // libera CPU
        }

        lonaFechada = true;
      }

      // se a chuva parar e o estado atual da lona for fechado, será aberto
      if (estado_chuva == 1 && lonaFechada) {
        Serial.println("Parou de chover! abrindo cobertura...");
        stepper.moveTo(stepper.currentPosition() - 2048); // abre a cobertura

        while (stepper.distanceToGo() != 0) {
          stepper.run();
          vTaskDelay(pdMS_TO_TICKS(1)); // libera CPU
        }

        lonaFechada = false;
      }

    } else {
      ESP_LOGE("Medição", "dados não disponíveis");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
};

// Task para enviar os dados por wi-fi
void vTaskComunicacao(void *pvParameters);