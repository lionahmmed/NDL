// /*
// Firmware_TTGO_Multi_Sensor_RSF_20241111_Deployed
// deviceName\\\":\\\"NDLNode #050609I0J789ND11\\\",  \\\"deviceSerial\\\":\\\"050609I0J789ND11
#define TINY_GSM_MODEM_SIM800
#include <Wire.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <ModbusRTU.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include "esp_bt.h"

const String deviceName = "NDLNode #050609I0J789ND11";
const String deviceSerial = "050609I0J789ND11";
const String recipientPhoneNumber = "+8801719258948";  // Replace with the actual recipient's phone number

#define RELAY 12
ModbusRTU mb;

#define SLAVE_ID_DO 01      // Example Slave ID for DO Sensor
#define SLAVE_ID_EC 02      // Example Slave ID for EC Sensor
#define SLAVE_ID_PH 03      // Example Slave ID for pH Sensor
#define SLAVE_ID_AMMONIA 4  //Example Slave ID for NH4 Sensor

#define RS485_DE_RE 32  // Control pin for DE/RE
HardwareSerial rs485(2);

#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define LED_GPIO 13
#define LED_ON HIGH
#define LED_OFF LOW

#define I2C_SDA 21
#define I2C_SCL 22

#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

const char apn[] = "blweb";  // APN for your network blweb
const char gprsUser[] = "";
const char gprsPass[] = "";

#define SerialMon Serial
#define TTGO Serial1

TinyGsm modem(TTGO);
TinyGsmClient client(modem);

unsigned long previousMillis = 0;       // Last time the 50-minute task was executed
const long long_interval = 60 * 60000;  // 50 minutes in milliseconds

bool setupPMU() {
  bool en = true;
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37);  // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35);  // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}

void setupModem() {
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LED_OFF);
}

void setupRS485() {
  rs485.begin(9600, SERIAL_8N1, 18, 19);
  mb.begin(&rs485, RS485_DE_RE);
  pinMode(RS485_DE_RE, OUTPUT);
  postTransmission();
  mb.master();  // Set ESP32 as Modbus master
}
void preTransmission() {
  digitalWrite(RS485_DE_RE, HIGH);  // Enable transmit mode
}
void postTransmission() {
  delay(10);
  digitalWrite(RS485_DE_RE, LOW);  // Enable receive mode
}


void powerUpmodem() {
  SerialMon.println("PowerUpModem");
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(6000);
  modem.restart();
  SerialMon.println("Initializing modem...");
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  if (modem.getSimStatus() != 3) {
    modem.simUnlock("");
    SerialMon.println(F("Modem Power ON"));
  }
}
void connectTonetwork() {
  SerialMon.print("Waiting for network...");
  if (modem.waitForNetwork()) {
    SerialMon.println(" success");
  } else {
    SerialMon.println(" fail. Retrying...");
    if (modem.waitForNetwork()) {
      SerialMon.println("Reconnection to network successful.");
    } else {
      SerialMon.println("Reconnection to network failed. Restarting Device...");
      ESP.restart();
    }
  }
}

void connectToInternet() {
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  bool connected = modem.gprsConnect(apn, gprsUser, gprsPass);
  if (connected) {
    SerialMon.println(" GPRS Connected");
  } else {
    SerialMon.println(" failed. Retrying to connect to the internet...");
    if (!modem.isNetworkConnected()) {
      SerialMon.println(F("Network not connected. Connecting to network..."));
      connectTonetwork();  // Connect to the network
    } else {
      SerialMon.println(F("Network already connected. Skipping network connection."));
    }
    connected = modem.gprsConnect(apn, gprsUser, gprsPass);
    if (connected) {
      SerialMon.println("Reconnection to the internet successful.");
    } else {
      SerialMon.println("Reconnection to the internet failed. Skipping this task.");
      return;  // Exit the function if the reconnection fails
    }
  }
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  } else {
    SerialMon.println("GPRS connection failed. Restarting the device...");
    ESP.restart();  // Restart the device to attempt a fresh connection
  }
}
String getFormattedTimeFromAPI() {
  SerialMon.println("Reading formatted time...");
  String formattedTime = "";
  bool success = false;
  for (int attempt = 0; attempt < 1; attempt++) {
    HttpClient timeClient(client, "worldtimeapi.org", 80);
    timeClient.get("/api/timezone/Asia/Dhaka");  // Bangladesh timezone
    int statusCode = timeClient.responseStatusCode();
    String response = timeClient.responseBody();
    if (statusCode == 200) {
      formattedTime = parseTimestampFromResponse(response);
      success = true;
      SerialMon.print("Formatted Time: ");
      SerialMon.println(formattedTime);
      break;
    } else {
      SerialMon.print("Failed to get time. HTTP error code: ");
      SerialMon.println(statusCode);
      delay(2000);
    }
  }
  if (!success) {
    SerialMon.println("Failed to get time after 2 attempts. Reconnecting to the Internet...");
    modem.gprsDisconnect();
    SerialMon.println(F("GPRS disconnected"));
    connectToInternet();
    for (int attempt = 0; attempt < 1 && !success; attempt++) {
      HttpClient timeClient(client, "worldtimeapi.org", 80);
      timeClient.get("/api/timezone/Asia/Dhaka");
      int statusCode = timeClient.responseStatusCode();
      String response = timeClient.responseBody();
      if (statusCode == 200) {
        formattedTime = parseTimestampFromResponse(response);
        success = true;
        SerialMon.print("Formatted Time: ");
        SerialMon.println(formattedTime);
      } else {
        SerialMon.print("Failed to get time after reconnection. HTTP error code: ");
        SerialMon.println(statusCode);
        delay(2000);
      }
    }
  }
  if (!success) {
    SerialMon.println("Failed to get time after all attempts.");
  }
  return formattedTime;
}
String parseTimestampFromResponse(String response) {
  int dateTimeIndex = response.indexOf("\"unixtime\":") + 11;
  String timestampStr = response.substring(dateTimeIndex, response.indexOf(",", dateTimeIndex));
  unsigned long timestamp = timestampStr.toInt();
  // Add 6 hours (6 hours * 60 minutes * 60 seconds = 21600 seconds)
  timestamp += 21600;  // Adjust for GMT +6
  time_t rawTime = timestamp;
  struct tm *timeInfo = localtime(&rawTime);
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%d/%m/%Y %H:%M:%S", timeInfo);
  return String(buffer);
}

void setup() {
  SerialMon.begin(115200);
  delay(10);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
  SerialMon.println("Wait...");
  TTGO.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  setupModem();
  powerUpmodem();
  setupRS485();
  pinMode(RELAY, OUTPUT);
}
void loop() {
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis >= long_interval) || previousMillis == 0) {
    previousMillis = currentMillis;  // Update the time marker
    SerialMon.println("Long Interval");
    digitalWrite(RELAY, HIGH);  // Execute your task here
    float WaterTemperature, WaterPh, ElectricalConductivity, DissolveOxygen, WaterAmmonia;
    readTemperatureSensor(WaterTemperature);
    readpHSensor(WaterPh);
    readECSensor(ElectricalConductivity);
    readDOSensor(DissolveOxygen);
    readAmmoniaSensor(WaterAmmonia);
    digitalWrite(RELAY, LOW);
    delay(100);
    if (!modem.isNetworkConnected()) {
      SerialMon.println(F("Network not connected. Connecting to network..."));
      connectTonetwork();  // Connect to the network
    } else {
      SerialMon.println(F("Network already connected. Skipping network connection."));
    }
        // Get the network signal strength (RSSI)
    int signalStrength = modem.getSignalQuality();
    SerialMon.print("Signal Strength (RSSI): ");
    SerialMon.println(signalStrength);
    // Format SMS message

    String smsMessage = "Device Serial: " + deviceSerial + "\n";
    smsMessage += "Temp: " + String(WaterTemperature, 2) + " C\n";
    smsMessage += "pH: " + String(WaterPh, 2) + "\n";
    smsMessage += "EC: " + String(ElectricalConductivity, 3) + " uS/cm\n";
    smsMessage += "DO: " + String(DissolveOxygen, 2) + " mg/L\n";
    smsMessage += "NH4: " + String(WaterAmmonia, 2) + " mg/L\n";
    smsMessage += "Signal Strength: " + String(signalStrength) + " dBm\n";

    // Send SMS
    sendSMS(smsMessage);
    connectToInternet();
    String formattedTime = getFormattedTimeFromAPI();
    String payloadAmmonia = constructPayload("Ammonia", "Water Ammonia", "mg/L", WaterAmmonia, formattedTime);
    sendData(payloadAmmonia);
    String payloadTemperature = constructPayload("Temperature", "Water Temperature", "C", WaterTemperature, formattedTime);
    sendData(payloadTemperature);
    String payloadDO = constructPayload("DissolvedOxygen", "Dissolved Oxygen", "mg/L", DissolveOxygen, formattedTime);
    sendData(payloadDO);
    String payloadEC = constructPayload("EC", "Electric Conductivity", "uS/cm", ElectricalConductivity, formattedTime);
    sendData(payloadEC);
    String payloadpH = constructPayload("pH", "Water pH", "", WaterPh, formattedTime);
    sendData(payloadpH);

    modem.gprsDisconnect();
    SerialMon.println(F("GPRS disconnected"));
      // digitalWrite(MODEM_POWER_ON, LOW);
      // SerialMon.println(F("Modem Power OFF"));
    delay(100);  // Small delay to ensure modem powers off properly
      // Set up timer wake-up (e.g., wake after 10 seconds (10 * 1000000)
  esp_sleep_enable_timer_wakeup(300 * 1000000); // Time in microseconds
  Serial.println("Going to deep sleep for 10 seconds...");
  esp_deep_sleep_start(); // Enter deep sleep mode
  }
}

// void sendSMS(const String &phoneNumber, const String &message) {
void sendSMS(const String &message) {
  SerialMon.println("Sending SMS...");
  // modem.sendSMS(phoneNumber, message);
  modem.sendSMS(recipientPhoneNumber, message);
  SerialMon.println("SMS sent successfully.");
}

// Function to construct payload for individual sensor readings
String constructPayload(String sensorType, String variableName, String units, float sensorValue, String formattedTime) {
  // String payload = "{\"payload\":\"{\\\"deviceName\\\":\\\"NDLNode #050609I0J789ND11\\\",  \\\"deviceSerial\\\":\\\"050609I0J789ND11\\\", \\\"valueVariable\\\":\\\"" + sensorType + "\\\", \\\"variableName\\\":\\\"" + variableName + "\\\", \\\"valueUnits\\\":\\\"" + units + "\\\", \\\"valueMeasure\\\":\\\"" + String(sensorValue, 3) + "\\\", \\\"ts\\\":\\\"" + formattedTime + "\\\"}\"}";
  String payload = "{\"payload\":\"{\\\"deviceName\\\":\\\"" + deviceName + "\\\",  \\\"deviceSerial\\\":\\\"" + deviceSerial + "\\\", \\\"valueVariable\\\":\\\"" + sensorType + "\\\", \\\"variableName\\\":\\\"" + variableName + "\\\", \\\"valueUnits\\\":\\\"" + units + "\\\", \\\"valueMeasure\\\":\\\"" + String(sensorValue, 3) + "\\\", \\\"ts\\\":\\\"" + formattedTime + "\\\"}\"}";
  SerialMon.print("Payload: ");
  SerialMon.println(payload);
  return payload;
}

void sendData(const String &payload) {
  bool dataSent = false;
  for (int attempt = 0; attempt < 2; attempt++) {
    HttpClient dataClient(client, "agritech-backend-api.nodesdigitalbd.com", 8083);  // Replace with your server domain and port
    dataClient.beginRequest();
    dataClient.post("/api/data/pipeline/v1/kafka/post/task/fisheries");  // Endpoint path
    dataClient.sendHeader("Content-Type", "application/json");
    dataClient.sendHeader("Content-Length", payload.length());
    dataClient.beginBody();
    dataClient.print(payload);
    dataClient.endRequest();
    int statusCode = dataClient.responseStatusCode();
    String response = dataClient.responseBody();
    if (statusCode == 200) {
      SerialMon.println("Data sent successfully.");
      dataSent = true;
      break;
    } else {
      SerialMon.print("Failed to send data. HTTP error code: ");
      SerialMon.println(statusCode);
      SerialMon.println(response);
      delay(2000);
    }
  }

  if (!dataSent) {
    modem.gprsDisconnect();
    SerialMon.println(F("GPRS disconnected"));
    SerialMon.println("Failed to send data after initial attempts. Reconnecting to internet...");
    connectToInternet();
    for (int retry = 0; retry < 2; retry++) {
      HttpClient dataClient(client, "agritech-backend-api.nodesdigitalbd.com", 8083);  // Replace with your server domain and port
      dataClient.beginRequest();
      dataClient.post("/api/data/pipeline/v1/kafka/post/task/fisheries");  // Endpoint path
      dataClient.sendHeader("Content-Type", "application/json");
      dataClient.sendHeader("Content-Length", payload.length());
      dataClient.beginBody();
      dataClient.print(payload);
      dataClient.endRequest();
      int statusCode = dataClient.responseStatusCode();
      String response = dataClient.responseBody();
      if (statusCode == 200) {
        SerialMon.println("Data sent successfully after reconnection.");
        dataSent = true;
        break;
      } else {
        SerialMon.print("Failed to send data after reconnection. HTTP error code: ");
        SerialMon.println(statusCode);
        SerialMon.println(response);
        delay(2000);
      }
    }
  }

  if (!dataSent) {
    SerialMon.println("Failed to send data after all attempts. Restarting the Device...");
    ESP.restart();
  }
}

void readTemperatureSensor(float &WaterTemperature) {
  Serial.println("Reading WaterTemperature sensor values...");
  uint16_t resultBuffer[2];
  uint16_t register_address = 0x2000;  // Start address for temperature data
  const int maxRetries = 5;            // Maximum number of retries
  bool success = false;                // Flag to indicate success
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    auto swapBytes = [](uint16_t value) -> uint16_t {
      return (value >> 8) | (value << 8);
    };
    preTransmission();
    delay(100);
    postTransmission();

    // Read 2 registers (4 bytes) from the DO sensor
    if (mb.readHreg(SLAVE_ID_DO, register_address, resultBuffer, 2)) {
      // Wait for response
      while (mb.slave()) {
        mb.task();
      }
      uint32_t rawValue = ((uint32_t)swapBytes(resultBuffer[1]) << 16) | swapBytes(resultBuffer[0]);
      memcpy(&WaterTemperature, &rawValue, sizeof(WaterTemperature));
      if (WaterTemperature > 0) {
        success = true;  // Set success to true if valid
        break;           // Exit the loop if a valid value is read
      } else {
        Serial.println("WaterTemperature value is zero or negative. Retrying...");
      }
    } else {
      Serial.println("Failed to read WaterTemperature.");
    }
    delay(2000);  // Wait before retrying
  }
  if (!success) {
    WaterTemperature = 0.0;  // Set to a default or error value
    Serial.println("Max retries reached. Setting WaterTemperature to 0.0 °C.");
  }
  Serial.print("WaterTemperature: ");
  Serial.print(WaterTemperature);
  Serial.println(" °C");
  delay(1000);
}

void readDOSensor(float &DissolveOxygen) {
  Serial.println("Reading DissolveOxygen sensor values...");
  uint16_t resultBuffer[2];
  uint16_t register_address = 0x2100;  // Start address for DO sensor data
  const int maxRetries = 5;            // Maximum number of retries
  bool success = false;                // Flag to indicate success
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    preTransmission();
    delay(100);
    postTransmission();
    if (mb.readHreg(SLAVE_ID_DO, register_address, resultBuffer, 2)) {
      while (mb.slave()) {
        mb.task();
      }
      uint32_t rawValue = ((uint32_t)((resultBuffer[1] >> 8) | (resultBuffer[1] << 8)) << 16) | ((resultBuffer[0] >> 8) | (resultBuffer[0] << 8));
      memcpy(&DissolveOxygen, &rawValue, sizeof(DissolveOxygen));
      if (DissolveOxygen > 0) {
        success = true;  // Set success to true if valid
        break;           // Exit the loop if a valid value is read
      } else {
        Serial.println("DissolveOxygen value is zero or negative. Retrying...");
      }
    } else {
      Serial.println("Failed to read DissolveOxygen.");
    }
    delay(2000);  // Wait before retrying
  }
  if (!success) {
    DissolveOxygen = 0.0;  // Set to a default or error value
    Serial.println("Max retries reached. Setting DissolveOxygen to 0.0 mg/L.");
  }
  Serial.print("DissolveOxygen: ");
  Serial.print(DissolveOxygen);
  Serial.println(" mg/L");
  delay(1000);
}

void readECSensor(float &ElectricalConductivity) {
  Serial.println("Reading ElectricalConductivity sensor values...");
  uint16_t resultBuffer[6];
  uint16_t register_address = 0x2004;  // EC sensor start address (adjust if different)
  const int maxRetries = 5;            // Maximum number of retries
  bool success = false;                // Flag to indicate success
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    preTransmission();
    delay(100);
    postTransmission();
    if (mb.readHreg(SLAVE_ID_EC, register_address, resultBuffer, 6)) {
      // Wait for response
      while (mb.slave()) {
        mb.task();
      }
      uint32_t rawData = ((uint32_t)((resultBuffer[3] >> 8) | (resultBuffer[3] << 8)) << 16) | ((resultBuffer[5] >> 8) | (resultBuffer[5] << 8));
      memcpy(&ElectricalConductivity, &rawData, sizeof(ElectricalConductivity));
      if (ElectricalConductivity > 0) {
        success = true;  // Set success to true if valid
        ElectricalConductivity *= 1000;
        break;  // Exit the loop if a valid value is read
      } else {
        Serial.println("ElectricalConductivity value is zero or negative. Retrying...");
      }
    } else {
      Serial.println("Failed to read ElectricalConductivity.");
    }
    delay(2000);  // Wait before retrying
  }
  if (!success) {
    ElectricalConductivity = 0.0;  // Set to a default or error value
    Serial.println("Max retries reached. Setting ElectricalConductivity to 0.0 mS/cm.");
  }
  Serial.print("ElectricalConductivity: ");
  Serial.print(ElectricalConductivity);
  Serial.println(" mS/cm");
  delay(1000);
}

void readpHSensor(float &WaterPh) {
  Serial.println("Reading WaterPh sensor values...");
  uint16_t resultBuffer[2];
  uint16_t register_address = 0x0601;  // pH sensor start address
  const int maxRetries = 5;            // Maximum number of retries
  bool success = false;                // Flag to indicate success
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    preTransmission();
    delay(100);
    postTransmission();
    if (mb.readHreg(SLAVE_ID_PH, register_address, resultBuffer, 2)) {
      while (mb.slave()) {
        mb.task();
      }
      uint32_t rawData = ((uint32_t)((resultBuffer[1] >> 8) | (resultBuffer[1] << 8)) << 16) | ((resultBuffer[1] >> 8) | (resultBuffer[1] << 8));
      memcpy(&WaterPh, &rawData, sizeof(WaterPh));
      if (WaterPh > 0) {
        success = true;  // Set success to true if valid
        break;           // Exit the loop if a valid value is read
      } else {
        Serial.println("WaterPh value is zero or negative. Retrying...");
      }
    } else {
      Serial.println("Failed to read WaterPh.");
    }
    delay(2000);  // Wait before retrying
  }
  if (!success) {
    WaterPh = 0.0;  // Set to a default or error value
    Serial.println("Max retries reached. Setting WaterPh to 0.0.");
  }
  Serial.print("WaterPh: ");
  Serial.println(WaterPh);
  delay(1000);
}
void readAmmoniaSensor(float &WaterAmmonia) {
  Serial.println("Reading WaterAmmonia sensor values...");
  uint16_t resultBuffer[4];
  uint16_t ion_concentration_address = 0x0000;  // Start address for Ion Concentration Value
  const int maxRetries = 5;                     // Maximum number of retries
  bool success = false;                         // Flag to indicate success
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    preTransmission();
    delay(100);
    postTransmission();
    if (mb.readHreg(SLAVE_ID_AMMONIA, ion_concentration_address, resultBuffer, 2)) {
      // Wait for response
      while (mb.slave()) {
        mb.task();
      }
      uint32_t rawValue = ((uint32_t)resultBuffer[1] << 16) | resultBuffer[0];
      memcpy(&WaterAmmonia, &rawValue, sizeof(WaterAmmonia));

      // const float SCALING_FACTOR = 0.35;
      // WaterAmmonia *= SCALING_FACTOR;

      if (WaterAmmonia > 0) {
        success = true;  // Set success to true if valid
        break;           // Exit the loop if a valid value is read
      } else {
        Serial.println("WaterAmmonia value is zero or negative. Retrying...");
      }
    } else {
      Serial.println("Failed to read WaterAmmonia.");
    }
    delay(2000);  // Wait before retrying
  }
  if (!success) {
    WaterAmmonia = 0.0;  // Set to a default or error value
    Serial.println("Max retries reached. Setting WaterAmmonia to 0.0 mg/L.");
  }
  Serial.print("WaterAmmonia: ");
  Serial.print(WaterAmmonia);
  Serial.println(" mg/L");
  delay(1000);
}
// */