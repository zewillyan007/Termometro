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
 
esp_adc_cal_characteristics_t adc_cal; //Estrutura que contem as informacoes para calibracao

//BluetoothSerial SerialBT; //Serial do bluetooth

//wifi usuario e senha
const char *ssid = "ESP32Will";
const char *password = "esp123456789";

WiFiClient client;
IPAddress ip (192, 168, 19, 2);
IPAddress netmask (255, 255, 255, 0);
const int port = 5210;
WiFiServer server(port);
 
//temperatura
bool esp32 = true;       // mude para falso ao usar o Arduino
int ThermistorPin;
double adcMax, Vs;
 
String TempReal;
double R1 = 10000.0;   // resitor 10k
double Beta = 3950.0;  // Beta value
double To = 295.15;    // temperatura em kelvin Kelvin
double Ro = 10000.0;   // resitor 10k em C
 
//calibrando tenção esp
//const float ADC_LUT[4096] PROGMEM =
//////////////////

int Conect;

void temperatura() {
    
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
  Tf = Tc * 9 / 5 + 32;              // Fahrenheit

  if (Tc > 0) {
      Serial.print(Tc);
      client.print(Tc);
    }

    Serial.println("°C");
    client.println("°C");
    delay(1000);
    // if (Tf > 0) Serial.print(Tf);
    // Serial.println(" Fahrenheit");
    // delay(1000);
}
 
void setup() {
  Serial.begin(9600);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0); //pin 34 esp32 devkit v1
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal); //Inicializa a estrutura de calibracao
 
 
  // temperatura
  ThermistorPin = 34;
  adcMax = 4095.0; // ADC 12-bit (0-4095)
  Vs = 3.3;        // voltagem

  //bluetooth
  //SerialBT.begin("ESP32 Will");

  //wifi
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

  if (!client.connected()) {
    Serial.println();
    Serial.println("Não há aparelho conectado");
    
    temperatura();
    Conect = 0;

    client = server.available();
    return;
  }

  if (client.available() >= 0) {

    if (Conect == 0) {
      Serial.println();
      Serial.println("Conectado com Sucesso");
    }
    
    temperatura();
    Conect ++;

  }
}