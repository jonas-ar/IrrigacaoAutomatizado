#include "ConfigManager.hpp"
#include <Preferences.h>

// Instancia o objeto para interagir com a NVS
Preferences preferences;

// Define um "namespace" para nossas configurações, para não misturar com outras libs
const char* PREFERENCES_NAMESPACE = "plantinha";

void setupConfig() {
    // Inicia o acesso à NVS no nosso namespace. O 'false' indica modo de leitura/escrita.
    preferences.begin(PREFERENCES_NAMESPACE, false);
}

bool loadConfig(DeviceConfig &config) {
    config.ssid = preferences.getString("ssid", "");
    config.password = preferences.getString("password", "");
    config.apiKey = preferences.getString("apiKey", "");
    config.serverUrl = preferences.getString("serverUrl", "");

    // Consideramos que a configuração é válida se a chave de API existir
    if (config.apiKey.length() > 0) {
        return true;
    }
    return false;
}

void saveConfig(const DeviceConfig &config) {
    preferences.putString("ssid", config.ssid);
    preferences.putString("password", config.password);
    preferences.putString("apiKey", config.apiKey);
    preferences.putString("serverUrl", config.serverUrl);
}

void clearConfig() {
    preferences.clear();
}