# Reto de Redes Industriales
Este repositorio contiene los códigos de los componentes del reto de búsqueda del tesoro, tanto el dispositivo de búsqueda 'master', como los dispositivos a buscar 'slaves'.

A continuación se detallan las funciones del 'master' para extraer datos usando diferentes protocolos de comunicación
## i2c
### Generalidades
El esclavo de i2c está configurado con una dirección `0x80`. Además de direcciones de registros `0x01`,`0x02`,`0x03`, y `0x04`, que corresponden respectivamente a **latitud**, **longitud**, **altitud**, y **latitud del objetivo final**

El módulo que contiene las funciones del master es `i2cMaster.h`
### Funciones
#### i2cRead(address)
Esta función escribe al registro correspondiente a una coordenada y entrega el valor respectivo.
- **Entradas**:
  - *address (uint8_t)*: La dirección del registro que dispara la lectura de la coordenada
- **Salidas**
  - *data (float)*: Valor del la coordenada
### Ejemplo de uso
```ino
#include <Wire.h>
#include <i2cMaster.h>
#define LAT_ADDRESS 0X01 //Ya están definidas en i2cMaster.h, pero es mejor dejarlas por claridad
#define LON_ADDRESS 0X02
#define ALT_ADDRESS 0X03
#define OBJ_LAT_ADDRESS 0X04

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  Serial.print("Latitud = ");
  latitud = i2cRead(LAT_ADDRESS);
  Serial.println(latitud, 4); //4 dígitos después del punto es suficiente presición para la tarea. Ver https://xkcd.com/2170/

  Serial.print("Longitud = ");
  longitud = i2cRead(LON_ADDRESS);
  Serial.println(longitud, 4);

  Serial.print("Altitud = ");
  altitud = i2cRead(ALT_ADDRESS);
  Serial.println(altitud, 4);

  Serial.print("Latitud Objetivo = ");
  latitudObjetivo = i2cRead(OBJ_LAT_ADDRESS);
  Serial.println(latitudObjetivo, 4);

  delay(1000);
}
```
## Bluetooth
### Generalidades
### Funciones
### Ejemplo de uso

## TCP/IP
### Generalidades
### Funciones
### Ejemplo de uso

## GPS_LCD
### V2.2 (Aun en prueba)
``ino
/////GPS
#include <HardwareSerial.h>
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);

//double latitud_destino = 6.1570398;  
//double longuitud_destino = -75.5162232;

//double latitud_destino = 6.1600829;  
//double longuitud_destino = -75.5156351;

double latitud_destino = 6.1542134;  
double longuitud_destino = -75.5199776;

double lat = 0;
double lon = 0;

double latDistance = 0;
double lonDistance = 0;
double totalDistance = 0;

int modo = 1;

const double EarthRadiusKm = 6371.0;
const double DegToRad = 0.017453292519943295; 

int satCuenta = 0;
/////PANTALLA LCD
#include <Wire.h>
#define I2C_ADDR 0x27 // Cambia 0x27 por la dirección I2C de tu LCD
#define BACKLIGHT 0x08 // Bit para controlar el backlight
#define En 0x04  // Bit de Enable
#define Rw 0x02  // Bit de Read/Write
#define Rs 0x01  // Bit de Register Select



void setup() {
  Serial.begin(9600);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  lcdInit();
  Wire.begin();
  delay(2000);
}
// PANTALLA
void escribirMenu(double lantitud_act,    double longuitud_act,
                  double Latitud_destino, double longuitud_destino,
                  double latDistance,     double lonDistance, 
                  double totalDistance,  int modo, int Sat ) {
  lcdSetCursor(0, 0);
  lcdPrint(String(lantitud_act,6) );
  lcdSetCursor(9, 0);
  lcdPrint(String(longuitud_act,6) );

  // Lat Long DESTINo
  lcdSetCursor(0, 1);
  lcdPrint(String(Latitud_destino,6) );
  lcdSetCursor(9, 1);
  lcdPrint(String(longuitud_destino,6) );
  // NORTE SUR ESTE OESTE
  lcdSetCursor(0, 2);
  lcdPrint("N:" + String(latDistance,0));

  lcdSetCursor(0, 3);
  lcdPrint("O:" + String(lonDistance,0));

  lcdSetCursor(13, 3);
  lcdPrint("MODE:" + String(modo));

  // DISTANCIA TOTAL, SATELITES
  lcdSetCursor(8, 2);
  lcdPrint("MG:" + String(totalDistance,0));
  lcdSetCursor(16, 2);
  lcdPrint("S:" + String(Sat));
}

void lcdSendNibble(byte data) {
  data |= BACKLIGHT; // Asegura que el backlight permanezca encendido
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(data | En); // Envía data con Enable en alto
  Wire.write(data & ~En); // Envía data con Enable en bajo
  Wire.endTransmission();
  delayMicroseconds(50); // Pequeña pausa para que el LCD procese los datos
}
void lcdSendByte(byte data, byte mode) {
  byte highNibble = data & 0xF0; // Obtiene el nibble alto
  byte lowNibble = (data << 4) & 0xF0; // Obtiene el nibble bajo
  lcdSendNibble(highNibble | mode); // Envía primero el nibble alto
  lcdSendNibble(lowNibble | mode); // Luego envía el nibble bajo
}
void lcdWriteChar(char c) {
  lcdSendByte(c, Rs);
}
void lcdSetCursor(byte col, byte row) {
  byte rowOffsets[] = {0x00, 0x40, 0x14, 0x54};
  lcdSendByte(0x80 | (col + rowOffsets[row]), 0);
}
void lcdInit() {
  Wire.begin();
  lcdSendNibble(0x03 << 4); // Modo de inicialización
  lcdSendNibble(0x03 << 4); // Modo de inicialización
  lcdSendNibble(0x03 << 4); // Modo de inicialización
  lcdSendNibble(0x02 << 4); // Habilita modo de 4 bits
  lcdSendByte(0x28, 0); // Función Set: modo de 4 bits, 2 líneas, 5x8 puntos
  lcdSendByte(0x0C, 0); // Display ON, Cursor OFF, Blink Cursor OFF
  lcdSendByte(0x06, 0); // Entry Mode Set
  lcdSendByte(0x01, 0); // Limpia pantalla
  delayMicroseconds(2000); // Espera a que el LCD procese el comando de limpiar
}
void lcdPrint(String str) {
  for (int i = 0; i < str.length(); i++) {
    lcdWriteChar(str[i]);
  }
}
void lcdSendCommand(byte cmd) {
  lcdSend(cmd, 0);
}
void lcdSendData(byte data) {
  lcdSend(data, Rs);
}
void lcdSend(byte value, byte mode) {
  byte high_nib = value & 0xF0;
  byte low_nib = (value << 4) & 0xF0;
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(high_nib | mode | BACKLIGHT);
  Wire.write(high_nib | En | mode | BACKLIGHT);
  Wire.write(high_nib | mode | BACKLIGHT);
  Wire.write(low_nib | mode | BACKLIGHT);
  Wire.write(low_nib | En | mode | BACKLIGHT);
  Wire.write(low_nib | mode | BACKLIGHT);
  Wire.endTransmission();
  delayMicroseconds(50);
}


//// GPS
void processGPRMC(String nmea) {
  // Parse the RMC sentence
  int startIndex = 0;
  int fieldIndex = 0;
  String fields[12];

  while (fieldIndex < 12 && startIndex < nmea.length()) {
    int endIndex = nmea.indexOf(',', startIndex);
    if (endIndex == -1) {
      fields[fieldIndex] = nmea.substring(startIndex);
      break;
    } else {
      fields[fieldIndex] = nmea.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
      fieldIndex++;
    }
  }

  char status = fields[2].charAt(0); // A for active, V for void

  if (status == 'A') {
    lat = convertNMEAToDegrees(fields[3]);
    lon = -1*convertNMEAToDegrees(fields[5]);
    float speed = fields[7].toFloat();
    String date = fields[9];

    Serial.print("Latitude: "); Serial.println(lat, 6);
    Serial.print("Longitude: "); Serial.println(lon, 6);
    Serial.print("Speed (knots): "); Serial.println(speed);
    Serial.print("Date: "); Serial.println(date);
  } else {
    Serial.println("Data is invalid");
  }
}
void processGPGGA(String nmea) {
  // Parse the GGA sentence
  int startIndex = 0;
  int fieldIndex = 0;
  String fields[15];

  while (fieldIndex < 15 && startIndex < nmea.length()) {
    int endIndex = nmea.indexOf(',', startIndex);
    if (endIndex == -1) {
      fields[fieldIndex] = nmea.substring(startIndex);
      break;
    } else {
      fields[fieldIndex] = nmea.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
      fieldIndex++;
    }
  }

  int satCount = fields[7].toInt();
  satCuenta = satCount;
  float altitude = fields[9].toFloat();
  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Satellites: "); Serial.println(satCount);
}
double convertNMEAToDegrees(String nmeaCoord) {
  if (nmeaCoord.length() < 6) {
    return 0.0;
  }
  int degreeIndex = nmeaCoord.indexOf('.') - 2;
  double degrees = nmeaCoord.substring(0, degreeIndex).toDouble();
  double minutes = nmeaCoord.substring(degreeIndex).toDouble();
  return degrees + minutes / 60.0;
}

void GPS(){
  String nmeaSentence = neogps.readStringUntil('\n');
  nmeaSentence.trim(); // Remove any trailing whitespace or newlines
      //escribirMenu(lat, lon, latitud_destino, longuitud_destino,latDistance, lonDistance, totalDistance, modo, satCuenta);
 
  if (nmeaSentence.startsWith("$GPRMC")) {
    processGPRMC(nmeaSentence);
    }
  if (nmeaSentence.startsWith("$GPGGA")) {
    processGPGGA(nmeaSentence);
    }
}

void calculateDistances(double lat1, double lon1, double lat2, double lon2,
                        double& latDistance, double& lonDistance, double& totalDistance) {
  // Convertir grados a radianes
  double lat1Rad = lat1 * DegToRad;
  double lon1Rad = lon1 * DegToRad;
  double lat2Rad = lat2 * DegToRad;
  double lon2Rad = lon2 * DegToRad;

  // Calcular diferencias
  double deltaLat = lat2Rad - lat1Rad;
  double deltaLon = lon2Rad - lon1Rad;

  // Calcular distancias en la superficie de la Tierra
  double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
             cos(lat1Rad) * cos(lat2Rad) * sin(deltaLon / 2) * sin(deltaLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  totalDistance = EarthRadiusKm * c * 1000; // Convertir km a metros

  // Diferencias de latitud y longitud en metros
  latDistance = deltaLat * EarthRadiusKm * 1000;
  lonDistance = deltaLon * EarthRadiusKm * cos((lat1Rad + lat2Rad) / 2) * 1000;

  // Ajustar signos para dirección
  latDistance = (latDistance > 0) ? fabs(latDistance) : -fabs(latDistance); // Norte positivo, Sur negativo
  lonDistance = (lonDistance > 0) ? fabs(lonDistance) : -fabs(lonDistance); // Este positivo, Oeste negativo
  Serial.print("Latitude Distance (m): "); Serial.println(latDistance);
  Serial.print("Longitude Distance (m): "); Serial.println(lonDistance);
  Serial.print("Total Distance (m): "); Serial.println(totalDistance);
}

void loop() {
  while (neogps.available()) {
    GPS();
    escribirMenu(lat, lon, latitud_destino, longuitud_destino,latDistance, lonDistance, totalDistance, modo, satCuenta);
    calculateDistances(lat, lon, latitud_destino, longuitud_destino,latDistance, lonDistance, totalDistance);
    }
}
```
