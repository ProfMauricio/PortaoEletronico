#include <Arduino.h>

/*****************************************************
 * Bibliotecas de MQTT e WiFi
 * **************************************************/
#include <PubSubClient.h>
#include <ESP8266WiFi.h>


/*****************************************************
 *  Biblioteca de controle do prompt de comando
 * **************************************************/
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>




/*****************************************************
 * Bibliotecas de EEPROM
 * **************************************************/
#include <EEPROM.h>


/*****************************************************
 * Bibliotecas pessoais
 * **************************************************/
#include "main.h"

// Objetos para parsear entrada via serial

CmdParser cmdParser;
CmdBuffer<64> cmdBuffer;
String bufferSerial = "buffer";
bool promptComandosAtivado = false;
unsigned long instanteInicial;
unsigned long instanteAtual;
unsigned long instanteInicialSensorPorta;
unsigned long instanteAtualSensorPorta;
bool ultimoEstadoPorta=false;





/********************************************************** 
 *    MQTT funções e variáveis
 *********************************************************/
//const char* BROKER_MQTT = "192.168.0.105"; // ip/host do broker
//int BROKER_PORT = 1883; // porta do broker
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient);
String topicoPortaIC = "PortaPrincipalIC";
String topicoStatusPorta = "StatusPortaPrincipalIC";
String topicoConectadoDispositivo = "StatusDispositivoPortaPrincipalIC";

int ledRede = D3;
int pinoSensorPorta = D2;
int ledPortaAberta = D4;

// armazena a situação de rede
int statusRede = WL_IDLE_STATUS; 
bool pisca = false;

long instanteUltimoEnvio = 0;  // TimeStamp da ultima mensagem enviada
const long TempoEspera = 5000;
int intervalo = 10000; 

/********************************************** 
 * Ajustes de conexão com a rede WiFi  
 *********************************************/ 
String ssid;
String pass;

// set wifissid=Sala21Dev
// set wifipass=#CAPp!10#
// set id=10 
const char Versao[] = "Versão 1.0 (Setembro/2022)";
const char Firmware[] = "Sistema de Controle de Porta do IC";
const char Separador[] = "*************************************************************";


/*********************************************************************************************
 *  Rotina para mostrar a porta serial funcionando e 
 *  as informações do software para o usuario
*********************************************************************************************/
void MsgInicializacao()
{
  delay(2000);
  Serial.println(Separador);
  Serial.println(Firmware);
  Serial.println(Versao);
  Serial.println(Separador);
  delay(3000);
}

/*********************************************************************************************
 * Rotina de conexão e status da conexão
*********************************************************************************************/

void ConectarWiFi()
{
  int contador=1;
  
  Serial.print(F("Conectando na rede WiFi "));
  Serial.print(ssid);
  Serial.print(F(" "));
  Serial.println("...");
  Serial.flush();
  while (statusRede != WL_CONNECTED )
  {    
    Serial.print(".");
    Serial.flush();
    digitalWrite(ledRede, LOW);
    pisca = !pisca;
    statusRede = WiFi.begin(ssid, pass);
    Serial.print("Tentativa ");
    Serial.println(contador);
    contador++;
    Serial.flush();    
    delay(15000);
  }
  digitalWrite(ledRede, HIGH);
  Serial.println("Saiu");
  Serial.print(F("Conectado a rede "));
  Serial.println(ssid);
  PrintWiFiConfigs();  
}

/*************************************************************************************
 * Mostra as configurações de rede, da placa e qualidade de sinal
 *************************************************************************************/
void PrintWiFiConfigs()
{
  byte mac[6];
  Serial.println(F("###### WiFi Configs #######"));
  Serial.print(F("IP: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("MAC: "));
  WiFi.macAddress(mac);
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

  IPAddress subnet = WiFi.subnetMask();
  Serial.print("NetMask: ");
  Serial.println(subnet);

  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("Gateway: ");
  Serial.println(gateway);

  Serial.println();
  Serial.print(F("Rede: "));
  Serial.println(WiFi.SSID());

  Serial.print(F("RSSI: "));
  Serial.println(WiFi.RSSI());
  Serial.println(F("###### WiFi Configs #######"));  
}

/*************************************************************************************
 * Mostra na serial os comandos disponveis
 *************************************************************************************/
void help()
{
  Serial.println(F("============================================================"));
  Serial.println(F("||                   Help de comandos                     ||"));
  Serial.println(F("============================================================"));
  Serial.println(F("|| set wifissid=<novoSSID>    |  Ajusta e salva novo SSID ||"));
  Serial.println(F("------------------------------------------------------------"));
  Serial.println(F("|| set wifipass=<novoPass>    |  Ajusta e salva nova PASS ||"));
  Serial.println(F("------------------------------------------------------------"));
  Serial.println(F("|| set id=<novoId>            |  Ajusta e salva novo ID   ||"));
  Serial.println(F("------------------------------------------------------------"));
  Serial.println(F("|| statusWiFi                 |  Mostra configs da rede   ||"));
  Serial.println(F("------------------------------------------------------------"));
  Serial.println(F("|| help                       |  Mostra help de comandos  ||"));
  Serial.println(F("============================================================"));
}

/*************************************************************************************
 * Funcao para obter os parametros passados pela serial
 *************************************************************************************/
String obterParametroSerial(int nroParam )
{
  String tmp="";
  tmp.reserve(5);
  switch (nroParam)
  {
  case 1:
    // formato esperado @salvar 24.2 60
    tmp = bufferSerial.substring(8,12);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 1 ->");
    Serial.println(tmp.toFloat());
#endif
    break;
  case 2:
    tmp = bufferSerial.substring(13,15);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 2 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  case 3:
    tmp = bufferSerial.substring(12,16);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 3 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  }
  return tmp;
} 

void tratarEntradaSerial()
{
  uint8_t par2;
  float par1;
  String tmpMsg;
  char parametro[30]; 
 
  // Parando timer para tratar informaçoes que estao sendo recebidas
  // -> não precisa.stop();

  // parando a interrupção
  //  MostrarInstanteAtual();
 if (cmdParser.parseCmd(cmdBuffer.getStringFromBuffer()) != CMDPARSER_ERROR) {
  Serial.print("Comando identificado: ");
  Serial.println(cmdParser.getCommand());
  Serial.print("Quantidade de parâmetros: ");
  Serial.println(cmdParser.getParamCount());

  if (cmdParser.equalCommand_P(PSTR("HELP")))
  {
    Serial.println("Ajuda ");
    help();
  }
  
  else if ( cmdParser.equalCommand_P(PSTR("STATUSWIFI")))
  {
    Serial.println(F("Exibindo status da rede WiFi"));
    PrintWiFiConfigs();
    Serial.println();
    Serial.print(F("Tempo desde o ultimo pacote: "));
    Serial.print((millis() - instanteUltimoEnvio)/1000.0);
    Serial.println(F("s"));
  }
  else if (cmdParser.equalCommand_P(PSTR("SET")))
  {
    Serial.println(F("Função de ajuste de configuração"));
    tmpMsg = cmdParser.getValueFromKey("WIFISSID"); 
    Serial.print("tmpMsg:");
    Serial.println(tmpMsg);
    if (tmpMsg.length() > 0) 
    {  
    Serial.print(F("Ajustando SSID da Rede sem Fio: "));
    Serial.println(tmpMsg);

    SaveConfigEEPROM("SSID", tmpMsg);
    }

    tmpMsg =  cmdParser.getValueFromKey_P(PSTR("WIFIPASS")); 
    Serial.print("tmpMsg:");
    Serial.println(tmpMsg);
    if (tmpMsg.length() > 0) 
    {  
    Serial.print(F("Ajustando Pass da Rede sem Fio: "));
    Serial.println(tmpMsg);
    SaveConfigEEPROM("PASS", tmpMsg);
    }

    Serial.println(cmdParser.getValueFromKey("ID"));
    Serial.print("tmpMsg:");
    tmpMsg = cmdParser.getValueFromKey_P(PSTR("ID"));
    Serial.println(tmpMsg);
    if (tmpMsg.length() > 0) 
    {  
    Serial.print(F("Ajustando Id do equipmamento: "));
    Serial.println(tmpMsg);
    SaveConfigEEPROM("ID", tmpMsg);
    }
    // implementar salvando na EEPROM
  }
  else if ( cmdParser.equalCommand_P(PSTR("RESET")))
  {
    Serial.print(F("Reset da memória"));
  }
  else 
    Serial.println(F("Comando não reconhecido"));

 }
  cmdBuffer.clear() ;

  /*
  else if ( tmpMsg.equalsIgnoreCase("@dado")) // usuário solicitando coleta de dados 
  {
    Serial.println(F("Ativando flag de captura"));    
    usuarioSolicitaLeitura = true;
    Serial.print("valor de flag: ");
    Serial.println(interruptFlagAlarm);
  }
  else if ( tmpMsg.equalsIgnoreCase("@tirq")) // requisição de instante pelo ESP
  {
    instante = rtc.GetDateTime();
    instanteStr = DateTime2String(instante);
    Serial1.print(instanteStr);
  } 
  else if (tmpMsg.equalsIgnoreCase("@help"))
    help();

  // testando se o comando para sair do modo de comandos
  else if (tmpMsg.equalsIgnoreCase("@exit"))
  {
    Serial.print(F("Saindo do modo de comando"));
  }
  else
  {
    tmpMsg = bufferSerial.substring(0,4);
    if (tmpMsg.equalsIgnoreCase("@now"))
    {
      instante = rtc.GetDateTime();
      instanteStr = DateTime2String(instante );
      Serial.println(instanteStr);
    }
    else if (tmpMsg.equalsIgnoreCase("@deb"))
    {
      Serial.println(F("Situacao do sistema"));
      instante = rtc.GetDateTime();
      instanteStr = DateTime2String(instante );
      Serial.println(instanteStr);
      Serial.print(F("Pulsos do pluviometro: "));
      Serial.println(contadorPulsosPluviometro);
      Serial.print("Umidade do ar (%): ");
      Serial.println(sensorHT.readHumidity());
      Serial.print("Temperatura (º): ");
      Serial.println(sensorHT.readTemperature()); 
    } 
    else 
       Serial.println(F("Comando nao reconhecido"));
  //*/
  // zerando buffer a cada comando
  bufferSerial = "";
}

/*************************************************************************************
 * Funçao para tratar os dados recebidos pela porta serial
 *************************************************************************************/
void serialEvent() {
  /*char inChar;
  //Serial.print("Analisando comando\n");
  while (Serial.available()) {
    // get the new byte:
    inChar = (char)Serial.read();
    // add it to the inputString:
    bufferSerial = bufferSerial + String(inChar);
    // Caractere @ inicia modo de prompt de comandos
    if ( (inChar == 10 ) || (inChar == 13) || (inChar == '#')) {
      Serial.print("bufferSerial: ");
      Serial.println(bufferSerial);
      tratarEntradaSerial();
    }
  }*/
  cmdBuffer.readFromSerial(&Serial,0);
  Serial.print("Comando Lido: ");
  Serial.print(cmdBuffer.getStringFromBuffer());
  bufferSerial = cmdBuffer.getStringFromBuffer();
  tratarEntradaSerial();
}


/********************************************************************************************
 *  Lendo configuracoes da EEPROM
 ********************************************************************************************/
void LoadConfigsEEPROM()
{
  EEPROM.begin(512);
  int temp = EEPROM.read(EEPROM_ID);
  ID_MQTT = temp;
  Serial.print("Id lido: ");
  Serial.println(ID_MQTT);

  //topicoConectado = "IC_Sala" + String(ID_MQTT) + "_online";
  //topicoTemperatura = "IC_Sala" + String(ID_MQTT) + "_Temp1"; 
  //topicoConectadoTemp = "PoP_Sala" + String(ID_MQTT) + "_Temp1_online";
  //topicoTemperatura = "PoP_Sala" + String(ID_MQTT) + "_Temp1"; 

  
 // novos topicos de Umidade
 //topicoConectadoUmid =  "PoP_Sala" + String(ID_MQTT) + "_Umid1_online";
 //topicoUmidade = "PoP_Sala" + String(ID_MQTT) + "_Umid1"; 
  int tamString = EEPROM.read(EEPROM_SSID);
  ssid="";
  for (int i = 1; i <= tamString; i++)
  {
    ssid += (char) EEPROM.read(EEPROM_SSID + i);
  }
  Serial.println(pass);

  tamString = EEPROM.read(EEPROM_PASS);
  Serial.print("Tamanho string pass: ");
  Serial.println(tamString);
  pass="";
  for (int i = 1; i <= tamString; i++)
  {
    pass += (char) EEPROM.read(EEPROM_PASS + i);
  }
  Serial.println(ssid);
  EEPROM.end();
}

/********************************************************************************************
 *  Salvando configurações na EEPROM
 ********************************************************************************************/
void SaveConfigEEPROM( String key, String value)
{
  int id;
  EEPROM.begin(512);
  if (key == "ID")
    {
      id = value.toInt();
      Serial.print("Id lido do parâmetro: ");
      Serial.println(id);
      EEPROM.write(EEPROM_ID, id );      
    }
  else if  (key == "SSID")
  {
    EEPROM.write(EEPROM_SSID, value.length() );
    for (int i = 1 ; i <= value.length(); i++ )
      EEPROM.write(EEPROM_SSID+i, (char) value[i-1]);

  }
   else if  (key == "PASS")
  {
    EEPROM.write(EEPROM_PASS, value.length() );
    for (int i = 1 ; i <= value.length(); i++ )
      EEPROM.write(EEPROM_PASS+i, (char) value[i-1]);
      
  } 
  EEPROM.end();
}



/**
 * @brief Função responsável por conectar ao Broker
 * 
 */
void ConectarMQTT()
{
  IPAddress server(200, 129, 247, 243);
  MQTT.setServer(server, 11883);   //informa qual broker e porta deve ser conectado  
  // Loop until we're reconnected
  while (!MQTT.connected()) {
    Serial.print("Tentando conectar ao broker ");
    Serial.println( BROKER_MQTT );
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    Serial.print("Cliente: ");
    Serial.println(clientId);
    // Attempt to connect
    if (MQTT.connect(clientId.c_str(),"ic", "#$?ic?#@")) {
      Serial.println("Conectado ao broker");  
      MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)  
      //MQTT.subscribe(topicoPortaIC.c_str());

    } 
    else {
      Serial.print("Falha, rc=");
      Serial.print(MQTT.state());
      Serial.println(" Tentando novamente em 5 segundos");
      // Wait 5 seconds before retrying
      delay(5000);
    }    
  }
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/ 
/**
 * @brief Callback para chegada de mensagens do broker
 * 
 * @param topic Topico que sofreu alteração
 * @param payload quantidade bytes da alteração
 * @param length tamanho do texto da alteração
 */
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  String msg;
  Serial.print("Message recebida [");
  Serial.print(topic);
  Serial.print("] ");
  //obtem a string do payload recebido
  for (unsigned int i = 0; i < length; i++)
  {
    char c = (char)payload[i];
    msg += c;
  }
  Serial.println(msg);
  TratarMsgRecebida(topic, msg );  
}




/**
 * 
 * 
 * ******************************************************************************/

/**
 * @brief Tratamento de tópicos alterados do MQTT
 * 
 * @param topic Topico alterado
 * @param msg Mensagem do tópico alterado
 */
void TratarMsgRecebida(String topic, String msg )
{
  Serial.println(F("Topico ativado"));
  if (( topic == topicoPortaIC ) && (msg == "abrir"))
   {
      Serial.println("Pedido de abertura de porta registrado");
      digitalWrite(D1,HIGH);
      delay(500);
      digitalWrite(D1,LOW);
      Serial.println("Sinal de abertura de porta enviado");      
      // publicando que está ativo      
      MQTT.publish(topicoPortaIC.c_str(), "fechar");      
   }
   

   /*
   if ( (topic = topic ) && (msg == "não")) 
     {
      Serial.println("Nova mensagem publicada Umid");
      // publicando que está ativo      
      MQTT.publish(topicoConectadoUmid.c_str(), "sim");
      ReportarInfoUsuario(ENVIANDO_INFO);
      flagSensorTemp1 = true;
      
      ReportarInfoUsuario(DESLIGAR_LED);
      
   } */
}

void setup() {
  // CONFIGURANDO PORTA DE ATIVAÇÃO DA  FECHADURA
  pinMode(D1,OUTPUT);
  // CONFIGURANDO A PORTA DO SENSOR DE FECHADO
  pinMode(pinoSensorPorta,INPUT);
  // CONFIGURANDO A PORTA DE REDE OK
  pinMode(ledRede,OUTPUT);
  // CONFIGURANDO A PORTA DE LED DE PORTA ABERTA
  pinMode(ledPortaAberta, OUTPUT);

  digitalWrite(D1,LOW);
  digitalWrite(ledRede,LOW);

  Serial.begin(9600);

  MsgInicializacao(); 
  help();
  Serial.println(F("Aguardando comandos da porta serial por 10 segundos")) ;
  instanteInicial = millis();
  instanteAtual = millis();
  cmdParser.setOptKeyValue(true);
  while ((instanteAtual - instanteInicial) < 10000 ) //20000)
  {
    cmdBuffer.readFromSerial(&Serial,5000);
    instanteAtual = millis();
    if (cmdBuffer.getBufferSize() > 0)
    {
      Serial.print("Comando Lido: ");
      Serial.println(cmdBuffer.getStringFromBuffer());
      bufferSerial = cmdBuffer.getStringFromBuffer();
      tratarEntradaSerial();
  }
   }

  
  LoadConfigsEEPROM() ; 
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println(F("Iniciando Conexão WiFi"));
  
  ConectarWiFi();


  Serial.print(F("Topico de verificação online é "));
  Serial.println(topicoConectadoDispositivo); 
    
  ConectarMQTT();
//  MQTT.publish(topicoTemperatura.c_str(),"30");
  MQTT.subscribe(topicoPortaIC.c_str());
  MQTT.publish(topicoConectadoDispositivo.c_str(), "sim");
  instanteAtualSensorPorta = millis();
}

void loop() {

 if (!WiFi.isConnected() )
    ConectarWiFi();    
  
  if (!MQTT.connected()) {
    ConectarMQTT();
    Serial.println("Conectado ao tópico " + topicoPortaIC);
    MQTT.subscribe(topicoPortaIC.c_str());
  }
    instanteAtualSensorPorta = millis();
   // Serial.println(instanteAtualSensorPorta - instanteInicialSensorPorta);
  if (( instanteAtualSensorPorta - instanteInicialSensorPorta) > 1000)
  {
      instanteInicialSensorPorta = millis();
      if (digitalRead(pinoSensorPorta) ==  HIGH)
      {
          if ( !ultimoEstadoPorta )
          {
              ultimoEstadoPorta = true;
              // publicar mensagem avisando da mudança de aberta para fechada              
              MQTT.publish(topicoStatusPorta.c_str(), "fechada"); 
          }
            
          Serial.println(" Porta fechada ");
          digitalWrite(ledPortaAberta, LOW);                    
       }
      else
      {
          Serial.println(" Porta aberta ");
          digitalWrite(ledPortaAberta, HIGH);                    
          if ( ultimoEstadoPorta )
          {
              ultimoEstadoPorta = false;
              // publicar mensagem avisando da mudança de fechada para aberta 
              MQTT.publish(topicoStatusPorta.c_str(), "aberta"); 
          }
      }
      
  }


  
  MQTT.loop();
  /*
  if ( digitalRead(D2) == HIGH)
    MQTT.publish(topicoStatusPorta.c_str(), "Fechada");   
  else
    MQTT.publish(topicoStatusPorta.c_str(), "Aberta");   
   
  */

}
