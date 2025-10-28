#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <string.h>
const char* ssid = "Realm of Chaos";
const char* password = "Kittehbuttzinwatah236";
std::string mqtt_username = "Ejolliffe4411";
std::string mqtt_password = "4411";
const char *mqtt_broker = "p07da41d.ala.us-east-1.emqxsl.com";

const int mqtt_port = 8883;


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
const char* z = "z";


const std::string mqtt_base = "SENG3030/Thursday/" + mqtt_username + "/";
const std::string mqtt_battery = mqtt_base + "battery";

const std::string mqtt_bmp_temperature = mqtt_base + "bmp280/temperature";
const std::string mqtt_bmp_pressure = mqtt_base + "bmp280/pressure";

const std::string mqtt_sht_temperature = mqtt_base + "sht40/temperature";
const std::string mqtt_sht_humidity = mqtt_base + "sht40/humidity";

const std::string mqtt_accel_x = mqtt_base + "accel/x";
const std::string mqtt_accel_y = mqtt_base + "accel/y";
const std::string mqtt_accel_z = mqtt_base + "accel/z";

const std::string mqtt_gyro_x = mqtt_base + "gyro/x";
const std::string mqtt_gyro_y = mqtt_base + "gyro/y";
const std::string mqtt_gyro_z = mqtt_base + "gyro/z";
WiFiClientSecure espClient;
PubSubClient client(espClient);

int mainLoopCounter;
const int DELAY_MQTT_PUBLISH = 1000000;//Loop Cycles
const int DELAY_DISPLAY_UPDATE = 5000;//Loop Cycles

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
int selection = 0;

void setup_unit_env(void);
void read_unit_env(void);
int get_Button_Presses(void);
void IMU_reading(void);
void setup_wifi(void);
void connectToMQTTBroker(void);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void send_mqtt_message(const std::string& topic, void* message);

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
  mainLoopCounter = 0;
}


void loop() {

  if(mainLoopCounter % DELAY_DISPLAY_UPDATE == 0)
  {
    M5.update();  // Required to get states
    M5.Display.setCursor(0, 40);
    M5.Display.clear();
    M5.Display.printf("Example Number: %d\r\n", 10);

    selection = get_Button_Presses();
    switch(selection)
    {
      case 1:
        read_unit_env();
        break;
      case 2:
        IMU_reading();
        break;
      case 3:
        M5.Speaker.tone(4000, 20);
        break;
      default:
        break;
    }
  }

  if(mainLoopCounter >= DELAY_MQTT_PUBLISH)
  {
    if (!client.connected()) {
        connectToMQTTBroker();
    }
    // Required for subscriptions to fire callback
    client.loop();

    if (!env_ok)
    {
      Serial.println("**** Unit ENV not set up ****");
      mainLoopCounter = 0;
      return;
    }
    read_unit_env();
    //Publish Battery Voltage to MQTT
    int voltage = M5.Power.getBatteryLevel();
    send_mqtt_message(mqtt_battery + "1", &voltage);
    sprintf(mqtt_buffer, "%d", voltage);
    client.publish(mqtt_battery.c_str(), mqtt_buffer);
    //Publish SHT40 to MQTT
    sprintf(mqtt_buffer, "%0.2f", sht4.cTemp);
    client.publish(mqtt_sht_temperature.c_str(), mqtt_buffer);
    sprintf(mqtt_buffer, "%0.2f", sht4.humidity);
    client.publish(mqtt_sht_humidity.c_str(), mqtt_buffer);
    //Publish BMP to MQTT
    sprintf(mqtt_buffer, "%0.2f", bmp.cTemp);
    client.publish(mqtt_bmp_temperature.c_str(), mqtt_buffer);
    sprintf(mqtt_buffer, "%0.2f", bmp.pressure);
    client.publish(mqtt_bmp_pressure.c_str(), mqtt_buffer);

    //Get IMU data and publish to MQTT
    auto IMU_update = M5.Imu.update();
    if (IMU_update) {
        auto data = M5.Imu.getImuData();
        //Publish Accel to MQTT
        sprintf(mqtt_buffer, "%0.2f", data.accel.x);
        client.publish(mqtt_accel_x.c_str(), mqtt_buffer);
        sprintf(mqtt_buffer, "%0.2f", data.accel.y);
        client.publish(mqtt_accel_y.c_str(), mqtt_buffer);
        sprintf(mqtt_buffer, "%0.2f", data.accel.z);
        client.publish(mqtt_accel_z.c_str(), mqtt_buffer);
        //Publish Gyro to MQTT
        sprintf(mqtt_buffer, "%0.2f", data.gyro.x);
        client.publish(mqtt_gyro_x.c_str(), mqtt_buffer);
        sprintf(mqtt_buffer, "%0.2f", data.gyro.y);
        client.publish(mqtt_gyro_y.c_str(), mqtt_buffer);
        sprintf(mqtt_buffer, "%0.2f", data.gyro.z); 
        client.publish(mqtt_gyro_z.c_str(), mqtt_buffer);
    }
    mainLoopCounter = 0;
  }
  mainLoopCounter++;
}

void send_mqtt_message(const std::string& topic, void* message) {
    sprintf(mqtt_buffer, "%0.2f", message);
    client.publish(topic.c_str(), mqtt_buffer);
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

  int voltage = M5.Power.getBatteryVoltage();
  Serial.print("Battery Voltage: ");
  Serial.println(voltage);
}

int get_Button_Presses(void)
{
  M5.update();  // Required to get states
  if (M5.BtnA.wasPressed()) {
    carsolCounter+= 1;

    if(carsolCounter > 4)
    {
      carsolCounter = 0;
    }  
    return 1;
  }
  if (M5.BtnB.wasPressed()) {
      return 2;
  }
  if (M5.BtnPWR.wasPressed()) {
      return 3;
  }
  return selection;
}

void IMU_reading(void){
  auto imu_update = M5.Imu.update();
    if (imu_update) {

        auto data = M5.Imu.getImuData();
        Serial.println("-----IMU-----");
        Serial.println("-----Accel-----");
        Serial.print("Accel X: ");
        Serial.println(data.accel.x);
        Serial.print("Accel Y: ");
        Serial.println(data.accel.y);
        Serial.print("Accel Z: ");
        Serial.println(data.accel.z);
        Serial.println("-----Gyro-----");
        Serial.print("Gyro X: ");
        Serial.println(data.gyro.x);
        Serial.print("Gyro Y: ");
        Serial.println(data.gyro.y);
        // The data obtained by getImuData can be used as follows.
        // data.accel.x;      // accel x-axis value.
        // data.accel.y;      // accel y-axis value.
        // data.accel.z;      // accel z-axis value.
        // data.accel.value;  // accel 3values array [0]=x / [1]=y / [2]=z.

        // data.gyro.x;      // gyro x-axis value.
        // data.gyro.y;      // gyro y-axis value.
        // data.gyro.z;      // gyro z-axis value.
        // data.gyro.value;  // gyro 3values array [0]=x / [1]=y / [2]=z.

        // data.value;  // all sensor 9values array [0~2]=accel / [3~5]=gyro /
        //              // [6~8]=mag

    }
}

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
            //client.publish((mqtt_base + "hello").c_str(), "Hello SENG3030!");
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

