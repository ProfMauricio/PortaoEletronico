#ifndef __MAIN_H__
#define  __MAIN_H__

#include <Arduino.h>

const int EEPROM_ID = 0;
const int EEPROM_SSID = 250;
const int EEPROM_PASS = 100; 
const unsigned int timeoutInicializacao = 10000;

char BROKER_MQTT[] ="200.129.247.243";
int BROKER_PORT = 1883;
int ID_MQTT = 84;

void PrintWiFiConfigs();
void MsgInicializacao();
void ConectarWiFi();
void help();
String obterParametroSerial(int nroParam );
void tratarEntradaSerial();
void LoadConfigsEEPROM();
void SaveConfigEEPROM( String key, String value);
/**
 * @brief Função responsável por conectar ao Broker
 * 
 */
void ConectarMQTT();
/**
 * @brief Callback para chegada de mensagens do broker
 * 
 * @param topic Topico que sofreu alteração
 * @param payload quantidade bytes da alteração
 * @param length tamanho do texto da alteração
 */
void mqtt_callback(char* topic, byte* payload, unsigned int length);
/**
 * @brief Tratamento de tópicos alterados do MQTT
 * 
 * @param topic Topico alterado
 * @param msg Mensagem do tópico alterado
 */
void TratarMsgRecebida(String topic, String msg );






#endif