#include "tasks.hpp"
#include "ultrassom_interrupcao.hpp"

// dados referente ao Wi-Fi
DeviceConfig deviceConfig;

// Definição das filas
QueueHandle_t fila_umidade_solo;
QueueHandle_t fila_chuva;
QueueHandle_t fila_nivel_agua;
QueueHandle_t fila_dados_irrigacao;
QueueHandle_t fila_comandos_remotos;

// Handles das tasks
TaskHandle_t handleSoloTask;
TaskHandle_t handleChuvaTask;
TaskHandle_t handleIrrigacaoTask;
TaskHandle_t handleNivelAgua;
TaskHandle_t handleComunicacao;

// define o mutex
SemaphoreHandle_t mutexDeviceConfig;

enum LonaControlState { AUTOMATIC, MANUAL_OVERRIDE };
LonaControlState lonaState = AUTOMATIC;
unsigned long manualOverrideStartTime = 0;
const unsigned long manualOverrideDuration = 3600000; // 1 hora em milissegundos

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

      leituras[i] = medirDistancia();

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

// Task para fazer o motor se movimetar
void vTaskMotor(void *pvParameters) {
  while (1) {
    stepper.run();
    esp_task_wdt_reset();  // alimenta o watchdog
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Task para realizar a irrigação
void vTaskIrrigacao(void *pvParameters) {
  int solo_medicao;
  int estado_chuva;
  float nivel_medicao;
  RemoteCommand cmd;

  // configurações do motor de passo
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(200);
  stepper.setSpeed(50);
  bool lonaFechada = false;
  DadosIrrigacao dados;
  dados.status_lona = 1;  // status padrão, 1 significa que a lona está aberta

  while(1) {
    if (xQueueReceive(fila_comandos_remotos, &cmd, (TickType_t)0) == pdTRUE) {
      Serial.printf("Comando recebido na vTaskIrrigacao: %s\n", cmd.action);
      if (strcmp(cmd.action, "WATER_PUMP") == 0) {
          Serial.println("Acionando a bomba de agua manualmente...");
          digitalWrite(PIN_RELE, HIGH);
          vTaskDelay(pdMS_TO_TICKS(5000)); // Liga por 5 segundos
          digitalWrite(PIN_RELE, LOW);
      } else if (strcmp(cmd.action, "TOGGLE_COVER") == 0) {
          Serial.println("Acionando a lona manualmente...");
          if (lonaFechada) {
              stepper.moveTo(stepper.currentPosition() - 4096); // Abre
          } else {
              stepper.moveTo(stepper.currentPosition() + 4096); // Fecha
          }
          lonaState = MANUAL_OVERRIDE; // Ativa o modo manual
          manualOverrideStartTime = millis(); // Marca o tempo
      }
    }

    // Verifica se o tempo de override manual já expirou
    if (lonaState == MANUAL_OVERRIDE && (millis() - manualOverrideStartTime > manualOverrideDuration)) {
        Serial.println("Override manual da lona expirou. Retornando ao modo automatico.");
        lonaState = AUTOMATIC;
    }

    // Tenta receber os dados dos sensores
    if (xQueueReceive(fila_umidade_solo, &solo_medicao, pdMS_TO_TICKS(10)) &&
        xQueueReceive(fila_chuva, &estado_chuva, portMAX_DELAY) &&
        xQueueReceive(fila_nivel_agua, &nivel_medicao, portMAX_DELAY)) {

        // Lógica de controle automático da lona (SÓ RODA NO MODO AUTOMÁTICO)
        if (lonaState == AUTOMATIC) {
            if (estado_chuva == 0 && !lonaFechada) { // Se está chovendo e a lona está aberta
                Serial.println("Chuva detectada. Fechando a lona automaticamente.");
                stepper.moveTo(stepper.currentPosition() + 4096); // Fecha a lona
                lonaFechada = true;
            }
            if (estado_chuva == 1 && lonaFechada) { // Se parou de chover e a lona está fechada
                Serial.println("Chuva parou. Abrindo a lona automaticamente.");
                stepper.moveTo(stepper.currentPosition() - 4096); // Abre a lona
                lonaFechada = false;
            }
        }
        
      // Status da lona baseado no estado atual físico.
        dados.status_lona = lonaFechada ? 0 : 1;

        
        dados.umidade_solo = solo_medicao;
        dados.nivel_agua = nivel_medicao;
        dados.estado_chuva = estado_chuva;

        if (nivel_medicao >= 21.0) {
        dados.status_bomba = 0; // 0 significa que a bomba está desligada
            digitalWrite(PIN_RELE, LOW);
        } else if (solo_medicao > 2300) {
        dados.status_bomba = 1; // 1 significa que a bomba está ligada
            digitalWrite(PIN_RELE, HIGH);
        } else {
        dados.status_bomba = 0; // 0 significa que a bomba está desligada
            digitalWrite(PIN_RELE, LOW);
        }

      // faz o controle do status de nível do solo
        if (solo_medicao > 2300) {
        dados.status_solo = 0; // solo seco
        } else if (solo_medicao >= 1850) {
        dados.status_solo = 1; // solo úmido
        } else {
        dados.status_solo = 2; // solo encharcado, crítico
        }

      // envia os dados para a fila
        xQueueSend(fila_dados_irrigacao, &dados, portMAX_DELAY);

    } else {
        ESP_LOGE("Medição", "dados nao disponíveis");
    }

    esp_task_wdt_reset(); // alimenta o watchdog
    vTaskDelay(pdMS_TO_TICKS(500));
  }
};

// Task para enviar os dados por wi-fi
void vTaskComunicacao(void *pvParameters) {
  DadosIrrigacao dados;
  
  while(1) {
    // Reconeção Wi-Fi se necessário
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-fi Desconectado. Tentando a reconexão...");
      WiFi.disconnect();

      if (xSemaphoreTake(mutexDeviceConfig, pdMS_TO_TICKS(100))) {
        WiFi.begin(deviceConfig.ssid.c_str(), deviceConfig.password.c_str());
        xSemaphoreGive(mutexDeviceConfig);
      }

      int tentativas = 0;
      while (WiFi.status() != WL_CONNECTED && tentativas < 10) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        tentativas++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nReconectado");
      } else {
        Serial.println("\nFalha na reconexão.");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
    }

    // Recebe dados da fila de irrigação
    if (xQueueReceive(fila_dados_irrigacao, &dados, portMAX_DELAY)) {
      Serial.println("Dados recebidos para envio por HTTP");
      dados.status_wifi = WiFi.RSSI();

      String postUrl;
      String apiKey;

      if (xSemaphoreTake(mutexDeviceConfig, pdMS_TO_TICKS(100))) {
        postUrl = deviceConfig.serverUrl;
        apiKey = deviceConfig.apiKey;
        xSemaphoreGive(mutexDeviceConfig);
      }

      postUrl.replace("/api", "");
      postUrl += "/api/device/sensor-readings";

      HTTPClient http;
      http.begin(postUrl.c_str());
      http.addHeader("Content-Type", "application/json");
      http.addHeader("x-api-key", apiKey.c_str());

      JsonDocument doc;
      doc["umidade"] = dados.umidade_solo;
      doc["chuva"] = dados.estado_chuva;
      doc["status_bomba"] = dados.status_bomba;
      doc["status_lona"] = dados.status_lona;
      doc["status_solo"] = dados.status_solo;
      doc["status_wifi"] = dados.status_wifi;
      doc["nivel_agua"] = dados.nivel_agua;

      // converte para string
      String json;
      serializeJson(doc, json);
      // envia por http
      int response = http.POST(json);

      if (response > 0) {
        Serial.printf("HTTP %d: %s\n", response, http.getString().c_str());
      } else {
        Serial.printf("ERRO HTTP %s\n", http.errorToString(response).c_str());
      }

      http.end();
    }

    // Comandos remotos
    String commandUrl;
    String apiKey;

    if (xSemaphoreTake(mutexDeviceConfig, pdMS_TO_TICKS(100))) {
      commandUrl = deviceConfig.serverUrl;
      apiKey = deviceConfig.apiKey;
      xSemaphoreGive(mutexDeviceConfig);
    }

    if (commandUrl.endsWith("/api")) {
      commandUrl += "/device/commands";
    } else {
      commandUrl += "/api/device/commands";
    }

    HTTPClient http_cmd;
    http_cmd.begin(commandUrl);
    http_cmd.addHeader("x-api-key", apiKey.c_str());

    int httpCodeCmd = http_cmd.GET();

    if (httpCodeCmd == HTTP_CODE_OK) {
      String payload = http_cmd.getString();
      Serial.println("Resposta de comandos recebida: " + payload);

      JsonDocument docCmd;
      deserializeJson(docCmd, payload);

      if (!docCmd["command"].isNull()) {
        RemoteCommand cmd;
        strlcpy(cmd.action, docCmd["command"]["action"], sizeof(cmd.action));
        cmd.value = docCmd["command"]["value"] | 0;

        if (xQueueSend(fila_comandos_remotos, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
          Serial.println("ERRO: Fila de comandos cheia!");
        } else {
          Serial.printf("Comando '%s' enviado para a fila de execucao.\n", cmd.action);
        }
      }
    } else if (httpCodeCmd > 0) {
      Serial.printf("Erro ao buscar comandos, HTTP %d\n", httpCodeCmd);
    } else {
      Serial.printf("Erro na conexao ao buscar comandos: %s\n", http_cmd.errorToString(httpCodeCmd).c_str());
    }

    http_cmd.end();

    esp_task_wdt_reset(); // alimenta o watchdog
    vTaskDelay(pdMS_TO_TICKS(10000)); // Aumentado para 10s para não sobrecarregar o servidor
  }
}