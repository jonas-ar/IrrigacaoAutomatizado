#ifndef PAIRING_MANAGER_HPP
#define PAIRING_MANAGER_HPP

#include "ConfigManager.hpp"

/**
 * @brief Tenta se conectar ao Wi-Fi com as credenciais salvas.
 * Se falhar, inicia um portal de configuração (portal cativo) para o usuário
 * inserir as novas credenciais e a API Key.
 * Esta é uma função bloqueante e deve ser chamada no setup().
 * @param config Referência para a struct de configuração que será preenchida.
 * @return true se a conexão for bem-sucedida, false se o portal foi iniciado e o dispositivo precisa reiniciar.
 */
bool setupAndAttemptConnection(DeviceConfig &config);

#endif