#ifndef CONFIG_MANAGER_HPP
#define CONFIG_MANAGER_HPP

#include <Arduino.h>

// Estrutura para armazenar todas as nossas configurações dinâmicas
struct DeviceConfig {
    String ssid;
    String password;
    String apiKey;
    String serverUrl;
};

/**
 * @brief Inicializa o sistema de Preferências (NVS).
 * Deve ser chamado uma vez no setup().
 */
void setupConfig();

/**
 * @brief Carrega a configuração salva na memória NVS.
 * @param config Referência para a struct onde a configuração será carregada.
 * @return true se uma configuração válida (com apiKey) foi encontrada, false caso contrário.
 */
bool loadConfig(DeviceConfig &config);

/**
 * @brief Salva a struct de configuração na memória NVS.
 * @param config A struct com os dados a serem salvos.
 */
void saveConfig(const DeviceConfig &config);

/**
 * @brief Limpa todas as configurações salvas na NVS.
 * Útil para forçar uma reconfiguração.
 */
void clearConfig();

#endif