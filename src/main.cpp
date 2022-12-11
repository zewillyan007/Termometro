#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
 
esp_adc_cal_characteristics_t adc_cal; //Estrutura de calibração

bool esp32 = true;       // mude para falso ao usar o Arduino

//Configuração WiFi
const char *ssid = "ESP32Will";
const char *password = "esp123456789";

WiFiClient client;
IPAddress ip (192, 168, 19, 2);
IPAddress netmask (255, 255, 255, 0);
const int port = 5210;
WiFiServer server(port);
 
//pinos, resistores e tensões
int ThermistorPin;
double adcMax, Vs;
 
String TempReal;
double R1 = 10000.0;   // resitor 10k
double Beta = 3950.0;  // Beta value
double To = 295.15;    // temperatura em kelvin Kelvin
double Ro = 10000.0;   // resitor 10k em sensor
 
//calibrando tenção esp

int Conect; //contador para verificar conexão

double Temperature() {
    
  uint32_t AD = 0;
  for (int i = 0; i < 100; i++)
  {
    AD += adc1_get_raw(ADC1_CHANNEL_6); //Obtem o valor RAW do ADC
    ets_delay_us(30);
  }
  AD /= 100;
 
  double Vout, Rt = 0;
  double T, Tc, Tf = 0;
 
  double adc = 0;
 
  adc = analogRead(ThermistorPin);
  adc = AD;
 
  Vout = adc * Vs / adcMax;
  Rt = R1 * Vout / (Vs - Vout);
 
  T = 1 / (1 / To + log(Rt / Ro) / Beta); //  Kelvin
  Tc = T - 273.15;                   // Celsius
  //Tf = Tc * 9 / 5 + 32;              // Fahrenheit

  return Tc;
}
 
void setup() {
  Serial.begin(9600);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0); //pin 34 esp32
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal); //Inicializa a estrutura de calibração
 
  ThermistorPin = 34; //pino do sensor
  adcMax = 4095.0; // ADC 12-bit (0-4095)
  Vs = 3.3;        // voltagem

  //WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask);
  WiFi.softAP(ssid, password);

  Serial.println("Wifi Details:");
  Serial.println(ssid);
  Serial.println(password);
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {

  double Temp = 0;

  if (!client.connected()) {
    Serial.println();
    Serial.println("Não há aparelho conectado!");
    
    Temp = Temperature();

    if (Temp > 0) {
      Serial.print(Temp);
      Serial.println("°C");
      delay(1000);
    }

    Conect = 0;

    client = server.available(); //reconexão
    return;
  }

  if (client.available() >= 0) {

    if (Conect == 0) {
      Serial.println();
      Serial.println("Conectado com Sucesso!");
    }
    
    Temp = Temperature();

    if (Temp > 0) {
      Serial.print(Temp);
      Serial.println("°C");
      client.print(Temp);
      client.println("°C");
      delay(1000);
    }

    Conect ++;
  }
}