#include <WiFi.h>
#include <WebServer.h>

// Configuración de la red WiFi
const char* ssid = "ESP32-AccessPoint";
const char* password = "12345678";

// Configuración de la IP estática
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Configuración del servidor web
WebServer server(80);

// Valores a enviar
String longitud = "123.456";
String latitud = "78.910";
String altitud = "101.112";
String latitud_final = "131.415";

void handleRoot() {
  String message = longitud + "," + latitud + "," + altitud + "," + latitud_final;
  server.send(200, "text/plain", message);
}

void setup() {
  Serial.begin(115200);

  // Configurar el ESP32 como punto de acceso con IP estática
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  Serial.println("Punto de acceso iniciado");
  Serial.print("Nombre de la red: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // Configurar el servidor web
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

void loop() {
  server.handleClient();
}
