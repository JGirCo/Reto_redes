#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);

double latitud_destino = 6.1565828;  
double longuitud_destino = -75.5182593;

double lat = 0;
double lon = 0;

double latDistance = 0;
double lonDistance = 0;
double totalDistance = 0;

int modo = 1;

const double EarthRadiusKm = 6371.0;
const double DegToRad = 0.017453292519943295; 

// Definir los protocolos
#define PROTOCOL_I2C 1
#define PROTOCOL_BLUETOOTH 2
#define PROTOCOL_TCP 3

// Pines
#define BUTTON_PIN 15  // Cambia esto por el pin en el que está conectado el pulsador

// Variables compartidas
float latitud = 0.0;
float longitud = 0.0;
float altitud = 0.0;
float latitudObjetivo = 0.0;
int satCuenta = 0;

//LCD
#define I2C_ADDR 0x27 // Cambia 0x27 por la dirección I2C de tu LCD
#define BACKLIGHT 0x08 // Bit para controlar el backlight
#define En 0x04  // Bit de Enable
#define Rw 0x02  // Bit de Read/Write
#define Rs 0x01  // Bit de Register Select

float lat1 = 0;
float lat2 = 0;
float lat3 = 0;
float lat4 = 0;

float lon1 = 0;
float lon2 = 0;
float lon3 = 0;
float lon4 = 0;

float alt1 = 0;
float alt2 = 0;
float alt3 = 0;
float alt4 = 0;


int protocoloSeleccionado = PROTOCOL_I2C;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Variables Bluetooth
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID_1("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID charUUID_2("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e");
static BLEUUID charUUID_3("12345678-1234-1234-1234-123456789abc");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

static BLEAdvertisedDevice* myDevice;
BLERemoteCharacteristic* pRemoteChar_1;
BLERemoteCharacteristic* pRemoteChar_2;
BLERemoteCharacteristic* pRemoteChar_3;

// Variables TCP
WiFiClient client;
bool wifiConnected = false;
const char* ssid = "ESP32-AccessPoint";
const char* password = "12345678";
const char* serverIP = "192.168.4.1";  // Dirección IP del ESP32 AP


void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                    uint8_t* pData,
                    size_t length,
                    bool isNotify) {
  if (pBLERemoteCharacteristic->getUUID().toString() == charUUID_1.toString()) {
    uint32_t counter = pData[0];
    for (int i = 1; i < length; i++) {
      counter = counter | (pData[i] << i * 8);
    }
    Serial.print("Characteristic 1 (Notify) from server: ");
    Serial.println(counter);
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice);
  Serial.println(" - Connected to server");

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  connected = true;
  pRemoteChar_1 = pRemoteService->getCharacteristic(charUUID_1);
  pRemoteChar_2 = pRemoteService->getCharacteristic(charUUID_2);
  pRemoteChar_3 = pRemoteService->getCharacteristic(charUUID_3);

  if (connectCharacteristic(pRemoteService, pRemoteChar_1) == false)
    connected = false;
  else if (connectCharacteristic(pRemoteService, pRemoteChar_2) == false)
    connected = false;
  else if (connectCharacteristic(pRemoteService, pRemoteChar_3) == false)
    connected = false;

  if (connected == false) {
    pClient->disconnect();
    Serial.println("At least one characteristic UUID not found");
    return false;
  }
  return true;
}

bool connectCharacteristic(BLERemoteService* pRemoteService, BLERemoteCharacteristic* l_BLERemoteChar) {
  if (l_BLERemoteChar == nullptr) {
    Serial.print("Failed to find one of the characteristics");
    Serial.print(l_BLERemoteChar->getUUID().toString().c_str());
    return false;
  }
  Serial.println(" - Found characteristic: " + String(l_BLERemoteChar->getUUID().toString().c_str()));

  if (l_BLERemoteChar->canNotify())
    l_BLERemoteChar->registerForNotify(notifyCallback);

  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

void setup() {
  
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  Serial.begin(115200);
  Serial.println("Starting Arduino Multi-Protocol application...");
  Wire.begin();
}

void loop() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading == HIGH && !buttonPressed && (millis() - lastDebounceTime) > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = millis();
    protocoloSeleccionado++;
    if (protocoloSeleccionado > PROTOCOL_TCP) {
      protocoloSeleccionado = PROTOCOL_I2C;
    }
    Serial.print("Protocolo seleccionado: ");
    Serial.println(protocoloSeleccionado);
  } else if (reading == LOW) {
    buttonPressed = false;
  }
  // if (protocoloSeleccionado == PROTOCOL_BLUETOOTH) {
  //   BLEDevice::init("");
  //   BLEScan* pBLEScan = BLEDevice::getScan();
  //   pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  //   pBLEScan->setInterval(1349);
  //   pBLEScan->setWindow(449);
  //   pBLEScan->setActiveScan(true);
  //   pBLEScan->start(5, false);
  // }

  if (protocoloSeleccionado == PROTOCOL_TCP && !wifiConnected) {
    WiFi.begin(ssid, password);
    Serial.print("Conectando a ");
    Serial.print(ssid);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi conectado");
    wifiConnected = true;
  }


  switch (protocoloSeleccionado) {
    case PROTOCOL_I2C:
      Serial.print("Latitud = ");
      latitud = i2cRead(0x01);
      Serial.println(latitud, 4);

      Serial.print("Longitud = ");
      longitud = i2cRead(0x02);
      Serial.println(longitud, 4);

      Serial.print("Altitud = ");
      altitud = i2cRead(0x03);
      Serial.println(altitud, 4);

      Serial.print("Latitud Objetivo = ");
      latitudObjetivo = i2cRead(0x04);
      Serial.println(latitudObjetivo, 4);
      break;

      //   case PROTOCOL_BLUETOOTH:
      //     if (doConnect) {
      //       if (connectToServer()) {
      //         Serial.println("We are now connected to the BLE Server.");
      //       } else {
      //         Serial.println("We have failed to connect to the server.");
      //       }
      //       doConnect = false;
      //     }

      //     if (connected) {
      //       std::string rxValue = pRemoteChar_1->readValue();
      //       Serial.print("Characteristic 1 (readValue): ");
      //       Serial.println(rxValue.c_str());

      //       rxValue = pRemoteChar_2->readValue();
      //       Serial.print("Characteristic 2 (readValue): ");
      //       Serial.println(rxValue.c_str());

      //       rxValue = pRemoteChar_3->readValue();
      //       Serial.print("Characteristic 3 (readValue): ");
      //       Serial.println(rxValue.c_str());
      //     } else if (doScan) {
      //       BLEDevice::getScan()->start(0);
      //     }
      //     break;

    case PROTOCOL_TCP:
      if (wifiConnected && client.connect(serverIP, 80)) {
        // Leer respuesta HTTP
        Serial.println("Conectado al servidor");

        // Enviar solicitud HTTP GET
        client.println("GET / HTTP/1.1");
        client.println("Host: " + String(serverIP));
        client.println("Connection: close");
        client.println();

        bool headersEnded = false;
        String response = "";
        while (client.connected() || client.available()) {
          if (client.available()) {
            String line = client.readStringUntil('\n');
            if (!headersEnded) {
              // Los encabezados HTTP terminan con una línea vacía
              if (line == "\r") {
                headersEnded = true;
              }
            } else {
              // Leer el cuerpo de la respuesta
              response += line;
            }
          }
        }

        Serial.println("Respuesta del servidor:");
        Serial.println(response);

        // Dividir la respuesta en los diferentes valores
        int index1 = response.indexOf(',');
        int index2 = response.indexOf(',', index1 + 1);
        int index3 = response.indexOf(',', index2 + 1);

        String strLongitud = response.substring(0, index1);
        String strLatitud = response.substring(index1 + 1, index2);
        String strAltitud = response.substring(index2 + 1, index3);
        String strLatitudFinal = response.substring(index3 + 1);

        // Convertir las cadenas a float
        longitud = strLongitud.toFloat();
        latitud = strLatitud.toFloat();
        altitud = strAltitud.toFloat();
        latitudObjetivo = strLatitudFinal.toFloat();

        Serial.println("Valores recibidos:");
        Serial.print("Longitud: ");
        Serial.println(longitud, 6);
        Serial.print("Latitud: ");
        Serial.println(latitud, 6);
        Serial.print("Altitud: ");
        Serial.println(altitud, 6);
        Serial.print("Latitud Final: ");
        Serial.println(latitudObjetivo, 6);

        client.stop();
      }
        else if (!wifiConnected) {
          Serial.println("Fallo al conectar al servidor: WiFi no está conectado");
        }
        delay(2000);  // Espera 2 segundos antes de la próxima conexión

        break;
      
  }
}
float i2cRead(uint8_t address) {
  Wire.beginTransmission(0x80);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x80, (uint8_t)4, (uint8_t) true);

  union {
    float floatValue;
    uint8_t bytes[4];
  } data;

  for (int i = 0; i < 4; i++) {
    data.bytes[i] = Wire.read();
  }

  return data.floatValue;
}
