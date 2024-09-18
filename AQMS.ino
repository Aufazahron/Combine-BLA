#include <SPI.h>
#include <SD.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <HardwareSerial.h>
#include "RTClib.h"

RTC_DS3231 rtc;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// UART parameters
#define RXD 25
#define TXD 27

// Command to read sensor
const uint8_t requestCommand[] = {0x3A, 0x10, 0x03, 0x00, 0x02, 0x02, 0x00, 0x00, 0x72, 0xEA}; 

// Pin untuk CS SD card module
const int chipSelect = 5; 
File logFile;

BLECharacteristic *pCharacteristic;
BLEServer *pServer;
BLEService *pService;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool sendData = false;
String startDate, endDate, startTime, endTime;

//Last ID
int getLastID;
float O3 = 0;
float NO2 = 0;
float SO2 = 0;
float NMHC = 0;
float NH3 = 0;
float H2S = 0;

// Callback class for handling connections and disconnections
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// Callback class for handling read and write requests
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.print("Received Value: ");
        Serial.println(value);
        // If the received value starts with "senddata"
        if (value.startsWith("senddata")) {
          int firstComma = value.indexOf(',');
          int secondComma = value.indexOf(',', firstComma + 1);
          int thirdComma = value.indexOf(',', secondComma + 1);
          int fourthComma = value.indexOf(',', thirdComma + 1);

          startDate = value.substring(firstComma + 1, secondComma);
          endDate = value.substring(secondComma + 1, thirdComma);
          startTime = value.substring(thirdComma + 1, fourthComma);
          endTime = value.substring(fourthComma + 1);

          sendData = true;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD, TXD);

  if (!rtc.begin()) {
    Serial.println("RTC tidak terdeteksi!");
    while (1);
  }

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Inisialisasi kartu SD gagal!");
    return;
  }
  Serial.println("Kartu SD berhasil diinisialisasi.");

  readLastID();
  Serial.print("Last ID: ");
  Serial.println(getLastID);

  // Open the log file in append mode
  logFile = SD.open("/sensor_data.csv", FILE_APPEND);
  if (!logFile) {
    Serial.println("Gagal membuka file");
  }

  // Create task for sensor reading
  xTaskCreatePinnedToCore(
    sensorTask,          // Function to be called
    "Sensor Task",       // Name of the task
    10000,               // Stack size (bytes)
    NULL,                // Parameter to pass
    1,                   // Task priority
    NULL,                // Task handle
    0                    // Core to run the task on
  );

  // Create task for log sending
  xTaskCreatePinnedToCore(
    logTask,             // Function to be called
    "Log Task",          // Name of the task
    10000,               // Stack size (bytes)
    NULL,                // Parameter to pass
    1,                   // Task priority
    NULL,                // Task handle
    1                    // Core to run the task on
  );
}

void loop() {
  // Main loop does nothing, tasks run independently
}

void sensorTask(void * parameter) {
  for (;;) {
    Readingsensor();
    readSensorData();
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

void logTask(void * parameter) {
  for (;;) {
    // Notify client if connected and sendData is true
    if (deviceConnected && sendData) {
      sendFile("/sensor_data.csv");
      sendData = false;  // Reset sendData after sending
    }

    // Check for device disconnection
    if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("Start advertising");
      oldDeviceConnected = deviceConnected;
    }

    // Check for new device connection
    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }

    // vTaskDelay(100 / portTICK_PERIOD_MS);  // Yield to other tasks
  }
}


// Function to read file from SD card and send via BLE
void sendFile(const char *filename) {
  File file = SD.open(filename);
  
  if (!file) {
    Serial.println("Gagal membuka file");
    return;
  }

  Serial.println("Membaca dan mengirim file:");
  
  String combinedData = "";
  int lineCount = 0;
  bool headerSkipped = false;
  
  while (file.available()) {
    String line = file.readStringUntil('\n');
    
    // Skip header
    if (!headerSkipped) {
      headerSkipped = true;
      continue;
    }
    
    // Parse the line
    int firstComma = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);
    int thirdComma = line.indexOf(',', secondComma + 1);
    int fourthComma = line.indexOf(',', thirdComma + 1);

    String date = line.substring(thirdComma + 1, fourthComma);
    String time = line.substring(fourthComma + 1, line.indexOf(',', fourthComma + 1));

    // Check if the date and time are within the specified range
    if ((date > startDate || (date == startDate && time >= startTime)) &&
        (date < endDate || (date == endDate && time <= endTime))) {
      
      String modifiedLine = "log," + line;
      combinedData += modifiedLine + "\n";
      lineCount++;
      
      // Send data if we have 3 lines or if it's the end of the file
      if (lineCount == 2 || !file.available()) {
        pCharacteristic->setValue(combinedData.c_str());
        pCharacteristic->notify();
        Serial.print("Sent Value: ");
        Serial.println(combinedData);
        combinedData = "";
        lineCount = 0;
        // delay(1); // delay to ensure data is sent correctly
      }
    }
  }
  
  file.close();
}

void Readingsensor(){
  Serial2.write(requestCommand, sizeof(requestCommand));
  
  uint8_t response[12];
  int len = Serial2.readBytes(response, 12);

  for (int i = 0; i < len; i++) {
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  if (len == 12) {
        // Check CRC
        unsigned short crc = modbus_CRC16(response, 10);
        if ((response[10] == (crc >> 8)) && (response[11] == (crc & 0xFF))) {
            // Extract the sensor value in ppb (4 bytes)
            uint32_t sensorValuePPB = (response[6] << 24) | (response[7] << 16) | (response[8] << 8) | response[9];
            
            // Convert ppb to ppm
            float sensorValuePPM = sensorValuePPB / 1000.0;

            // Print the sensor value
            Serial.print("Raw sensor value (ppb): ");
            Serial.println(sensorValuePPB);
            Serial.print("Sensor value (ppm): ");
            Serial.println(sensorValuePPM);
            NH3 = sensorValuePPM;
        } else {
            Serial.println("CRC check failed");
        }
    } else {
        Serial.println("Incorrect response length");
    }
}

void readSensorData() {
  getLastID++;

  // Simulasi pembacaan sensor, gantikan dengan pembacaan sensor yang sebenarnya
  O3 = random(25, 35) + 0.1 * random(0, 10);
  NO2 = random(10, 20) + 0.1 * random(0, 10);
  SO2 = random(5, 15) + 0.1 * random(0, 10);
  NMHC = random(0, 1) + 0.01 * random(0, 100);
  // NH3 = random(0, 1) + 0.01 * random(0, 100);
  H2S = random(0, 1) + 0.01 * random(0, 100);

  String data = String(getLastID) + ",Kota Bandung,AQMS PRT," + getTimeDate() + 
                "," + String(O3) + "," + String(NO2) + "," + String(SO2) + 
                "," + String(NMHC) + "," + String(NH3) + "," + String(H2S) + "\n";
  String dataBLE = "live," + String(data);
  
  Serial.print("Logged data: ");
  Serial.println(data);

  logFile.print(data);
  logFile.flush();

  pCharacteristic->setValue(dataBLE.c_str());
  pCharacteristic->notify();
}

String getTimeDate(){
  DateTime now = rtc.now();
  char timeString[9];
  char dateString[11];
  sprintf(dateString, "%04d-%02d-%02d", now.year(), now.month(), now.day());
  sprintf(timeString, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  String data = String(dateString) + "," + String(timeString);
  return data;
}

String getDateString() {
  // Simulasi tanggal, gantikan dengan tanggal yang sebenarnya
  return "2024-07-05";
}

// Function to get the current time as a string
String getTimeString() {
  // Simulasi waktu, gantikan dengan waktu yang sebenarnya
  return "00:00:00";
}

// Function to calculate CRC16
unsigned short modbus_CRC16(unsigned char *ptr, unsigned char len) {
    unsigned short wcrc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        wcrc ^= *ptr++;
        for (int j = 0; j < 8; j++) {
            if (wcrc & 0x0001) {
                wcrc = wcrc >> 1 ^ 0xA001;
            } else {
                wcrc >>= 1;
            }
        }
    }
    return (wcrc << 8) | (wcrc >> 8);
}

// Read Last ID
void readLastID() {
  File file = SD.open("/sensor_data.csv");
  if (!file) {
    Serial.println("Gagal membuka file");
  }

  long fileSize = file.size();  // Dapatkan ukuran file
  long position = fileSize - 1; // Mulai dari akhir file

  // Baca file dari belakang ke depan
  String lastLine = "";
  while (position >= 0) {
    file.seek(position);
    char c = file.read();
    if (c == '\n') {
      // Jika menemukan new line (\n), kita sudah menemukan baris terakhir
      if (lastLine.length() > 0) {
        break;
      }
    } else {
      // Tambahkan karakter ke lastLine
      lastLine = c + lastLine;
    }

    position--;
  }

  file.close();

  // Pisahkan lastLine untuk mendapatkan ID
  int commaIndex1 = lastLine.indexOf(',');
  String lastID = lastLine.substring(0, commaIndex1);  // ID adalah bagian pertama sebelum koma pertama
  getLastID = lastID.toInt();
}

