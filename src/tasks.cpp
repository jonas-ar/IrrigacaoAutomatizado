#include "tasks.hpp"

// dados referente ao Wi-Fi
const char* ssid = "NOME_REDE_WIFI";
const char* senha = "SENHA_REDE_WIFI";
const char* url = "http://exemplo.servidor/dados";

// Definição das filas
QueueHandle_t fila_umidade_solo;
QueueHandle_t fila_chuva;
QueueHandle_t fila_nivel_agua;
QueueHandle_t fila_dados_irrigacao;

// Handles das tasks
TaskHandle_t handleSoloTask;
TaskHandle_t handleChuvaTask;
TaskHandle_t handleIrrigacaoTask;
TaskHandle_t handleNivelAgua;
TaskHandle_t handleComunicacao;

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
  const int numLeituras = 5;
  long duracao;
  float leituras[numLeituras];

  while(1) {
    for (int i = 0; i < numLeituras; i++) {
      digitalWrite(PIN_TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(PIN_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_TRIG, LOW);

      duracao = pulseIn(PIN_ECHO, HIGH);
      leituras[i] = duracao * 0.034 / 2;

      vTaskDelay(pdMS_TO_TICKS(50)); // intervalo entre leituras
    }

    // ordena o vetor para obter a mediana
    std::sort(leituras, leituras + numLeituras); // adapta o algoritmo de ordenação com base no tamanho do vetor

    float mediana;
    if (numLeituras % 2 == 0) {
      mediana = (leituras[numLeituras / 2 - 1] + leituras[numLeituras / 2]) / 2.0;
    } else {
      mediana = leituras[numLeituras / 2];
    }

    xQueueSend(fila_nivel_agua, &mediana, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// instância do motor
AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);

// Task para realizar a irrigação
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

      // Serial.printf("Valor bruto do sensor de umidade do solo: %d \n", solo_medicao);
      // Serial.printf("Valor bruto do sensor de chuva (0 chovendo, 1 sem chover): %d \n", estado_chuva);
      // Serial.printf("Distância em cm: %f\n\n", nivel_medicao);

      if (nivel_medicao >= 21.0) {
        // Serial.println("Nível de água CRÍTICO, a bomba será desligada.");
        digitalWrite(PIN_RELE, LOW);
      } else if (solo_medicao > 2500) {
        // Serial.println("Solo seco detectado. Bomba ligada para irrigação.");
        digitalWrite(PIN_RELE, HIGH);
      } else {
        // Serial.println("Solo úmido. Bomba desligada.");
        digitalWrite(PIN_RELE, LOW);
      }
    
      // se estiver chovendo e o estado atual da lona for aberta, será fechado
      if (estado_chuva == 0 && !lonaFechada) {
        // Serial.println("Está chovendo! acionando cobertura...");
        stepper.moveTo(stepper.currentPosition() + 2048); // fecha a cobertura

        while (stepper.distanceToGo() != 0) {
          stepper.run();
          vTaskDelay(pdMS_TO_TICKS(1)); // libera CPU
        }

        lonaFechada = true;
      }

      // se a chuva parar e o estado atual da lona for fechado, será aberto
      if (estado_chuva == 1 && lonaFechada) {
        // Serial.println("Parou de chover! abrindo cobertura...");
        stepper.moveTo(stepper.currentPosition() - 2048); // abre a cobertura

        while (stepper.distanceToGo() != 0) {
          stepper.run();
          vTaskDelay(pdMS_TO_TICKS(1)); // libera CPU
        }

        lonaFechada = false;
      }

      // envia os dados para a task de comunicação
      DadosIrrigacao dados; // faz a instância
      dados.umidade_solo = solo_medicao;
      dados.nivel_agua = nivel_medicao;
      dados.estado_chuva = estado_chuva;

      xQueueSend(fila_dados_irrigacao, &dados, portMAX_DELAY);

    } else {
      ESP_LOGE("Medição", "dados não disponíveis");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
};

// Task para enviar os dados por wi-fi
void vTaskComunicacao(void *pvParameters) {
  DadosIrrigacao dados;
  
  // faz a conexão com wi-fi
  WiFi.begin(ssid, senha);
  Serial.print("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado.");
  
  while(1) {
    if (xQueueReceive(fila_dados_irrigacao, &dados, portMAX_DELAY)) {
      Serial.println("Dados recebidos para envio por HTTP");

      HTTPClient http;
      http.begin(url);
      http.addHeader("Content-Type", "application/json");

      // monta o json
      String json = String("{\"umidade\":") + dados.umidade_solo +
                    ",\"chuva\":" + dados.estado_chuva +
                    ",\"nivel_agua\":" + dados.nivel_agua + "}";

      int response = http.POST(json);

      if (response > 0) {
        Serial.printf("HTTP %d: %s\n", response, http.getString().c_str());
      } else {
        Serial.printf("ERRO HTTP %s\n", http.errorToString(response).c_str());
      }

      http.end();
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}