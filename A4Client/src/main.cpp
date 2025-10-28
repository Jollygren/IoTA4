#include "M5Unified.h"

// Include required for the ENV IV module
#include <M5UnitENV.h>

// I2C Pins for the external I2C port
#define EXT_SDA 32
#define EXT_SCL 33

// SHT40 Humidity and Temperature I2C sensor
SHT4X sht4;
// BMP280 Pressure and Temperature I2C sensor
BMP280 bmp;
// Flag that ENV was set up
bool env_ok = false;

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <string.h>
const char* ssid = "Barney188";
const char* password = "Bindernagel8";
std::string mqtt_username = "Jkaufman2743";
std::string mqtt_password = "2743";
const char *mqtt_broker = "p07da41d.ala.us-east-1.emqxsl.com";

const char* topicDefault = "SENG3030/Thursday/Ejolliffe4411/#";
int carsolCounter = -1;

const char* battery = "battery";
const char* sht40 = "sht40";
const char* bmp280 = "bmp280";
const char* temperature = "temperature";
const char* humidity = "humidity";
const char* pressure = "pressure";
const char* accel = "accel";
const char* gyro = "gyro";
const char* x = "x";
const char* y = "y";
const char* z = "z";

const int mqtt_port = 8883;


const std::string mqtt_base = "SENG3030/Thursday/" + mqtt_username + "/";
const std::string mqtt_battery = mqtt_base + "battery";

const std::string mqtt_bmp_temperature = mqtt_base + "bmp280/temperature";
const std::string mqtt_bmp_pressure = mqtt_base + "bmp280/pressure";

const std::string mqtt_sht_temperature = mqtt_base + "sht40/temperature";
const std::string mqtt_sht_humidity = mqtt_base + "sht40/humidity";
WiFiClientSecure espClient;
PubSubClient client(espClient);

char mqtt_buffer[40];

const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTTBroker() {
    while (!client.connected()) {
        String client_id = "esp32-client-" + String(WiFi.macAddress());
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username.c_str(), mqtt_password.c_str())) {
            Serial.println("Connected to MQTT broker");
            client.subscribe((mqtt_base + "#").c_str());
            // Publish message upon successful connection
            client.publish((mqtt_base + "hello").c_str(), "Hello SENG3030!");
        } else {
            Serial.print("Failed to connect to MQTT broker, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(25000);
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message received on topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
//-------------------------------------------------STUFF I ADDED---------------------------
switch (carsolCounter)
    {
    case 0: // BATTERY----------------------------------------

    if(strstr(topic, battery) != NULL)
      {
        M5.Display.clear();
        M5.Display.setCursor(10, 30);
        M5.Display.print("BAT: %");
        for (unsigned int i = 0; i < length; i++) 
        {
          M5.Display.print((char) payload[i]);
        }
      }
      break;
    
    case 1: // TEMP & HUMD-------------------------------------
      if(strstr(topic, sht40) != NULL)
      {
        if(strstr(topic, temperature) != NULL)
        {
          M5.Display.clear();
          M5.Display.setCursor(10, 30);
          M5.Display.print("Temperature: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }

         if(strstr(topic, humidity) != NULL)
        {
          M5.Display.println();
          M5.Display.print("Humidity: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }
      }
      break;

    case 2: // TEMP AND PRESS-----------------------------------
      if(strstr(topic, bmp280) != NULL)
      {
        if(strstr(topic, temperature) != NULL)
        {
          M5.Display.clear();
          M5.Display.setCursor(10, 30);
          M5.Display.print("Temperature: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }

         if(strstr(topic, pressure) != NULL)
        {
          M5.Display.println();
          M5.Display.print("Pressure: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }
      }
      break;
    
    case 3: // ACCEL------------------------------------------
      if(strstr(topic, accel) != NULL)
      {
        if(strstr(topic, x) != NULL)
        {
          M5.Display.clear();
          M5.Display.setCursor(10, 30);
          M5.Display.print("Accel X: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }

        int temp = strlen(topic);
        if(topic[temp-1] =='y')
        {
          M5.Display.println();
          M5.Display.print("Accel Y: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }

        if(strstr(topic, z) != NULL)
        {
          M5.Display.println();
          M5.Display.print("Accel Z: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }
      }
      break;
    
    case 4: // GYRO
      if(strstr(topic, gyro) != NULL)
      {
        if(strstr(topic, x) != NULL)
        {
          M5.Display.clear();
          M5.Display.setCursor(10, 30);
          M5.Display.print("Gyro X: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }

        int temp = strlen(topic);
        if(topic[temp-1] =='y')
        {
          M5.Display.println();
          M5.Display.print("Gyro Y: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }

        if(strstr(topic, z) != NULL)
        {
          M5.Display.println();
          M5.Display.print("Gyro Z: ");
          for (unsigned int i = 0; i < length; i++) 
          {
            M5.Display.print((char) payload[i]);
          }
        }
      }
      break;
    
    
    default:
      break;
    }
   
}





void setup_unit_env(void);
void read_unit_env(void);

void setup() {
  Serial.begin(115200);
  Serial.println("** Starting Setup **");

  // Set up the ENV Sensor
  setup_unit_env();

  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.setTextColor(GREEN);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setFont(&fonts::FreeSansBold9pt7b);
  M5.Display.setTextSize(1);


  setup_wifi();

  // Set Root CA certificate
  espClient.setCACert(ca_cert);

  client.setServer(mqtt_broker, mqtt_port);
  client.setKeepAlive(60);
  client.setCallback(mqttCallback);
  connectToMQTTBroker();
  client.subscribe(topicDefault);

}

void loop() {
 
  //Serial.println("** Running Loop **"); -------------LOOP START-------------------------
  //read_unit_env();



//----------------------------------------------------BUTTONS-----------------------------------
  M5.update();  // Required to get states
if (M5.BtnA.wasPressed()) {
    Serial.println("A Btn Pressed");

    carsolCounter+= 1;

    if(carsolCounter > 4)
    {
      carsolCounter = 0;
    }
}
if (M5.BtnA.wasReleased()) {

    Serial.println("A Btn Released");
}
if (M5.BtnB.wasPressed()) {
    Serial.println("B Btn Pressed");
}
if (M5.BtnB.wasReleased()) {
    Serial.println("B Btn Released");
}
if (M5.BtnPWR.wasClicked()) {
    Serial.println("PWR Btn Clicked");
}
if (M5.BtnPWR.wasHold()) {
    Serial.println("PWR Btn Hold");
}





    if (!client.connected()) {
        connectToMQTTBroker();
    }
    // Required for subscriptions to fire callback
    client.loop();

  delay(1000);
}

void setup_unit_env()
{
  Serial.println("** Setting Up Unit ENV **");

  // IMPORTANT - for the M5Stick-C Plus2, the pins we need are 32 and 33,
  // not what the example at github uses
  if (!sht4.begin(&Wire, SHT40_I2C_ADDR_44, EXT_SDA, EXT_SCL, 400000U)) {
      Serial.println("Couldn't find SHT4x");
      while (1) delay(1);
}

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  if (!bmp.begin(&Wire, BMP280_I2C_ADDR, EXT_SDA, EXT_SCL, 400000U)) {
      Serial.println("Couldn't find BMP280");
      while (1) delay(1);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                  BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  BMP280::FILTER_X16,      /* Filtering. */
                  BMP280::STANDBY_MS_500); /* Standby time. */  
  
  env_ok = true;
}

void read_unit_env(void)
{
  if (not env_ok)
  {
    Serial.println("**** Unit ENV not set up ****");
    return;
  }

  if (sht4.update()) {
          Serial.println("-----SHT4X-----");
          Serial.print("Temperature: ");
          Serial.print(sht4.cTemp);
          Serial.println(" degrees C");
          Serial.print("Humidity: ");
          Serial.print(sht4.humidity);
          Serial.println("% rH");
          Serial.println("-------------\r\n");
  }

  if (bmp.update()) {
      Serial.println("-----BMP280-----");
      Serial.print(F("Temperature: "));
      Serial.print(bmp.cTemp);
      Serial.println(" degrees C");
      Serial.print(F("Pressure: "));
      Serial.print(bmp.pressure);
      Serial.println(" Pa");
      Serial.print(F("Approx altitude: "));
      Serial.print(bmp.altitude);
      Serial.println(" m");
      Serial.println("-------------\r\n");
  }
}
