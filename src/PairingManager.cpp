#include "PairingManager.hpp"
#include <WiFiManager.h>

bool shouldSaveConfig = false;

// Função de callback que o WiFiManager chamará quando o usuário salvar os dados no portal
void saveConfigCallback() {
  Serial.println("Callback de salvamento acionado. Novas credenciais para salvar.");
  shouldSaveConfig = true;
}

bool setupAndAttemptConnection(DeviceConfig &config) {
    WiFiManager wm;

    // Cria os campos customizados para o nosso portal
    WiFiManagerParameter custom_api_key("apiKey", "API Key", config.apiKey.c_str(), 64);
    WiFiManagerParameter custom_server_url("serverUrl", "URL do Servidor", config.serverUrl.c_str(), 128);

    // Adiciona os campos ao portal
    wm.addParameter(&custom_api_key);
    wm.addParameter(&custom_server_url);

    // Define a função de callback a ser chamada no salvamento
    wm.setSaveParamsCallback(saveConfigCallback);

    // Tenta se conectar. Se não conseguir, inicia o portal com o nome "MinhaPlantinha-Setup"
    // O portal ficará ativo por 5 minutos. Se o usuário não fizer nada, o ESP reinicia.
    wm.setConfigPortalTimeout(300);

    if (!wm.autoConnect("MinhaPlantinha-Setup")) {
        Serial.println("Falha ao conectar e o tempo do portal expirou. Reiniciando...");
        delay(3000);
        ESP.restart(); 
        return false;
    }

    Serial.println("Conectado ao Wi-Fi!");

    // Se o callback foi acionado, significa que novos dados foram inseridos no portal
    if (shouldSaveConfig) {
        Serial.println("Salvando nova configuracao...");
        // Pega os valores dos campos customizados
        config.apiKey = custom_api_key.getValue();
        config.serverUrl = custom_server_url.getValue();
        // O SSID e a senha são salvos automaticamente pelo WiFiManager na memória do Wi-Fi
        // Aqui pegamos eles para salvar em nosso ConfigManager também
        config.ssid = WiFi.SSID();
        config.password = WiFi.psk();

        saveConfig(config);

        Serial.println("Configuracao salva. O dispositivo sera reiniciado.");
        delay(3000);
        ESP.restart();
        return false;
    }

    return true; // Conexão bem-sucedida com credenciais já existentes
}