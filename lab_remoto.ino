/*
  PINES:

  LED 1: 4 / D4
  LED 2: 13 / D7
  LED 3: 2 / D9
  BUZZER: 14 / D5
  DHT(TEMP): 5 / D3
  SERVO: 16 / D2
  RELE: 0 / D8

*/


#include "PubSubClient.h"
#include <ArduinoJson.h>
#ifdef ESP32
  #include <ESP32Servo.h>  // Normal para ESP32
  #include <WiFi.h>
#else
  #include <Servo.h>  // Si estás usando esta versión
  #include <ESP8266WiFi.h>
  extern "C" {
  #include <user_interface.h>
}
#endif

#include "DHT.h" // Incluimos la librería para trabajar con el sensor DHT22
#define DHT_PIN 5  // Pin donde está conectado el sensor DHT22
#define DHTTYPE DHT22   // Definimos el tipo de sensor DHT
DHT dht(DHT_PIN, DHTTYPE);

#define LED1_PIN 4  // Pin donde está conectado el LED
#define LED2_PIN 13 // Pin donde está conectado el LED
#define LED3_PIN 2  // Pin built in de la placa
#define VIB_PIN 25 //Sensor de vibracion
#define MIC_PIN 35 //Sensor microfono

#define BUZZER_PIN 14
#define RELE_PIN 0

#define TRIG_PIN 18 // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 5 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin

#define PIR_PIN 26

#define MQ2_ANA_PIN 34

#define LDR_PIN 32 // ESP32 pin GIOP32(ADC0)

//float LDR_LUX_VALUE = 0;
int LDR_VALUE = 0;
float MIC_VALUE;
int MOV_VALUE=0;
int PIR_VALUE;
int VIB_VALUE;
float SR04_VALUE;
int MQ2_VALUE;
float DTH22_TEMP_VALUE;
float DTH22_HUM_VALUE;


//mqtt seccion 
const int SERVO_PIN = 16;
Servo servo;

const char* mqttServer = "lrfia.uai.edu.ar";
int port = 6033;
const char* mqtt_user = "lab_remoto_iot_ypf";
const char* mqtt_pass = "Axc45+23_";

//const char * board_id = "AQF4556"; //placa 1
//8uint8_t board_mac[6] = {0xDE, 0xAD, 0x01, 0xC4, 0x0A, 0x21};
const char * board_id = "AQF4352";  //placa 2
uint8_t board_mac[6] = {0xDE, 0xAD, 0x01, 0xC4, 0x0A, 0x22};
//const char * board_id = "AQF4573"; //placa 3
//uint8_t board_mac[6] = {0xDE, 0xAD, 0x01, 0xC4, 0x0A, 0x23};
//const char * board_id = "AQF4254"; //placa 4
//uint8_t board_mac[6] = {0xDE, 0xAD, 0x01, 0xC4, 0x0A, 0x24};
//const char * board_id = "AQF4155"; //placa 5
//uint8_t board_mac[6] = {0xDE, 0xAD, 0x01, 0xC4, 0x0A, 0x25};
//const char * board_id = "AQF4576"; //placa 6
//uint8_t board_mac[6] = {0xDE, 0xAD, 0x01, 0xC4, 0x0A, 0x26};


String dashboard_topic_sub_str = String(board_id) + "_pub"; //se susbscribe para escuchar lo que publica la placa
String evaluador_topic_ack_str = String(board_id) + "_ack"; //publica para el evaluador la confirmación de que la placa recibio la orden
String dashboard_topic_pub_str = String(board_id) + "_sub";//publica en sub que la placa esta escuchando

const char* dashboard_topic_sub = dashboard_topic_sub_str.c_str();
const char* evaluador_topic_ack = evaluador_topic_ack_str.c_str();
const char* dashboard_topic_pub = dashboard_topic_pub_str.c_str();

String stMac;
char mac[50];
char clientId[50];

WiFiClient espClient;
PubSubClient client(espClient);


// Lista de posibles redes
const char* redes[][2] = {
  {"LAB. ROBOTICA", "LabRoboticaUAI"},
  {"UAI-FI", "AccesosUAI"},
  {"InnovativaLab", ""},
  {"Wokwi-GUEST", ""},
  {"lrfia_uai", "123456+"},
};
const int NUM_REDES = sizeof(redes) / sizeof(redes[0]);
// Variables globales para recordar la red usada en esta sesión
String last_ssid = "";
String last_pass = "";



int mode_board = 0;  // 0: normal, 1: modo inyección
unsigned long tiempo_inicio_inyeccion = 0;
const unsigned long TIEMPO_MAX_INYECCION = 5 * 60 * 1000; // 5 minutos en milisegundos
bool temporizador_activo = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("IoT board connected!");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Inicializamos el sensor DHT22 para empezar a leer la temperatura
  dht.begin();  

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  pinMode(PIR_PIN, INPUT);
  pinMode(VIB_PIN, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELE_PIN, OUTPUT);

  // Intentar obtener la MAC antes de conectar WiFi
  WiFi.mode(WIFI_STA);
  delay(100); // Pequeña pausa para que el WiFi se inicialice
  
  // Asignar MAC según plataforma
   // #ifdef ESP32
    //WiFi.setMacAddress(board_mac);
   //   esp_wifi_set_mac(WIFI_IF_STA,board_mac);
   // #elif defined(ESP8266)
   //   wifi_set_macaddr(STATION_IF, board_mac);
   // #endif

  wifiConnect();

  Serial.println("");
  Serial.println("WiFi connected: ");
  Serial.println(last_ssid);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // La MAC ya se obtiene en wifiConnect() de forma robusta
  
  client.setServer(mqttServer, port);
  client.setCallback(callback);
  
  servo.attach(SERVO_PIN, 500, 2400);

}
//---------------
void wifiConnect() {
  bool conectado = false;

  // Si ya se conectó antes en esta ejecución, reintentar con esa
  if (last_ssid != "") {
    Serial.printf("Intentando con red previa: %s\n", last_ssid.c_str());
    WiFi.begin(last_ssid.c_str(), last_pass.c_str());
    if (esperarConexion()) {
      Serial.println("Conectado con red previa.");
      conectado = true;
    } else {
      Serial.println("Fallo con red previa.");
    }
  }

  // Si no hay red previa o falló, probar la lista completa
  if (!conectado) {
    for (int i = 0; i < NUM_REDES; i++) {
      Serial.printf("Intentando conectar a %s...\n", redes[i][0]);
      WiFi.begin(redes[i][0], redes[i][1]);
      if (esperarConexion()) {
        last_ssid = redes[i][0];
        last_pass = redes[i][1];
        conectado = true;
        break;
      }
    }
  }

  if (!conectado) {
    Serial.println("No se pudo conectar a ninguna red.");
  }

}

bool esperarConexion() {
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}


void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    long r = random(1000);
    sprintf(clientId, "clientId-%ld", r);

    // Conexión con usuario y contraseña
    if (client.connect(clientId, mqtt_user, mqtt_pass)) {
      Serial.print(clientId);
      Serial.println(" connected");
      client.subscribe(dashboard_topic_sub);
     // publicar_mqtt(dashboard_topic_pub,"mensaje", "Conectado ok!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publicar_ack(const char* device, const char* value){
  // Create JSON document
  DynamicJsonDocument jsonDoc(1024);
  jsonDoc["command"] = "data_ack";
  jsonDoc["device"] = device;
  jsonDoc["value"] = value; // Replace this with your actual sensor value

  // Serialize JSON document to a string
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  // Publish the JSON data to the "esp32/json" topic
  client.publish(evaluador_topic_ack, jsonString.c_str());
}

void publicar_data_sensor(const char* device, const char* value){
  // Create JSON document
  DynamicJsonDocument jsonDoc(1024);
  jsonDoc["command"] = "data_sensor";
  jsonDoc["device"] = device;
  jsonDoc["value"] = value; // Replace this with your actual sensor value

  // Serialize JSON document to a string
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  // Publish the JSON data to the "esp32/json" topic
  client.publish(dashboard_topic_pub, jsonString.c_str());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String stMessage;
  char buffer[16];
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    stMessage += (char)message[i];
  }
  Serial.println();

  if (String(topic) == dashboard_topic_sub) {
     // Crear un objeto JSON
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, stMessage);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    //activa mode board
    if (doc["command"] == "enviar" && doc["device"] == "board mode") {
      if (doc["value"] == "1") {
        mode_board = 1;
        tiempo_inicio_inyeccion = millis();
        temporizador_activo = true;
        Serial.println("Modo inyección activado");
      } else  if (doc["value"] == "0") {
        mode_board = 0;
        temporizador_activo = false;
        Serial.println("Modo normal activado manualmente");
      }
    }

    //modo inyeccion los sensores se cargan desde el backend
    if (mode_board && doc["command"] == "inyectar"){
        if(doc["device"] == "ldr") {
          LDR_VALUE = doc["value"];
        }else if (doc["device"]=="sr04"){
          SR04_VALUE = doc["value"];
        }
        else if (doc["device"]=="mov"){
          MOV_VALUE = doc["value"];
        }
        else if (doc["device"]=="pir"){
          PIR_VALUE  = doc["value"];
        }
        else if (doc["device"]=="mq2"){
          MQ2_VALUE = doc["value"];
        }
        else if (doc["device"]=="temp"){
          DTH22_TEMP_VALUE = doc["value"];
        }
        else if (doc["device"]=="hum"){
          DTH22_HUM_VALUE = doc["value"];
        }
        else if (doc["device"]=="vib"){
          VIB_VALUE  = doc["value"];
        }
        else if (doc["device"]=="mic"){
          MIC_VALUE  = doc["value"];
        }
    }

    if (doc["command"]=="enviar"){

      if (doc["device"]=="led1"){
          Serial.print("Led 1 ");
          if(doc["value"] == "1"){
            Serial.println("on");
            digitalWrite(LED1_PIN, HIGH);
            publicar_ack("led1", "1");
          }else {
            Serial.println("off");
            digitalWrite(LED1_PIN, LOW);
            publicar_ack("led1", "0");
          }
      }else if(doc["device"]=="led2"){
          Serial.print("Led 2 ");
          if(doc["value"] == "1"){
            Serial.println("on");
            digitalWrite(LED2_PIN, HIGH);
            publicar_ack("led2", "1");
          }else {
            Serial.println("off");
            digitalWrite(LED2_PIN, LOW);
            publicar_ack("led2", "0");
          }
      }else if(doc["device"]=="led3"){
          Serial.print("Led 2 ");
          if(doc["value"] == "1"){
            Serial.println("on");
            digitalWrite(LED3_PIN, HIGH);
            publicar_ack("led2", "1");
          }else {
            Serial.println("off");
            digitalWrite(LED3_PIN, LOW);
            publicar_ack("led2", "0");
          }
      }else if (doc["device"]=="rel"){
          Serial.print("Rele ");
          if(doc["value"] == "1"){
            Serial.println("on");
            digitalWrite(RELE_PIN, HIGH);
            publicar_ack("rel", "1");
          }else {
            Serial.println("off");
            digitalWrite(RELE_PIN, LOW);
            publicar_ack("rel", "0");
          }
      }else if (doc["device"]=="srv"){
          //float ang = atoi(doc["value"]);
          float ang = doc["value"].as<float>();
          servo.write(ang);
      }else if (doc["device"]=="buz"){
          Serial.print("Buzzer ");
          if(doc["value"] == "1"){
            for (int i = 0; i <= 200; i++) {
              digitalWrite(BUZZER_PIN, HIGH);
              delayMicroseconds(400);
              digitalWrite(BUZZER_PIN, LOW);
              delay(2);
            }
            Serial.println("on");
            publicar_ack("buz", "1");
          }else {
            Serial.println("off");
            digitalWrite(BUZZER_PIN, LOW);
            publicar_ack("buz", "0");
          }
      }
    }
    else if (doc["command"]=="leer"){
        
        if (doc["device"]=="ldr"){
          sprintf(buffer, "%d", LDR_VALUE);
          publicar_data_sensor("ldr", buffer);
          publicar_ack("ldr", buffer);
        }
        else if (doc["device"]=="sr04"){
          dtostrf(SR04_VALUE, 6, 2, buffer);
          publicar_data_sensor("sr04", buffer);
          publicar_ack("sr04", buffer);
        }
        else if (doc["device"]=="mov"){
          sprintf(buffer, "%d", MOV_VALUE);
          publicar_data_sensor("mov", buffer);
          publicar_ack("mov", buffer);
        }
        else if (doc["device"]=="pir"){
          sprintf(buffer, "%d", PIR_VALUE);
          publicar_data_sensor("pir", buffer);
          publicar_ack("pir", buffer);
        }
        else if (doc["device"]=="mq2"){
          sprintf(buffer, "%d", MQ2_VALUE);
          publicar_data_sensor("mq2", buffer);
          publicar_ack("mq2", buffer);
        }
        else if (doc["device"]=="temp"){
          dtostrf(DTH22_TEMP_VALUE, 6, 2, buffer);
          publicar_data_sensor("temp", buffer);
          publicar_ack("temp", buffer);
        }
        else if (doc["device"]=="hum"){
          dtostrf(DTH22_HUM_VALUE, 6, 2, buffer);
          publicar_data_sensor("hum", buffer);
          publicar_ack("hum", buffer);
        }
        else if (doc["device"]=="vib"){
          dtostrf(VIB_VALUE, 6, 2, buffer);
          publicar_data_sensor("vib", buffer);
          publicar_ack("vib", buffer);
        }
        else if (doc["device"]=="mic"){
          dtostrf(MIC_VALUE, 6, 2, buffer);
          publicar_data_sensor("mic", buffer);
          publicar_ack("mic", buffer);
        }
        Serial.print(doc["device"].as<String>() ); 
        Serial.print(": "); 
        Serial.println(buffer);   // the raw analog reading
    }
  }
}


//-----------------
void leer_MIC(){
  MIC_VALUE = analogRead(MIC_PIN);
}
void leer_VIB(){
  VIB_VALUE = digitalRead(VIB_PIN);
}
void leer_PIR(){
  PIR_VALUE = digitalRead(PIR_PIN);
}
void leer_DTH22(){
   // Leemos la temperatura desde el sensor DHT22
  DTH22_TEMP_VALUE = dht.readTemperature();
  DTH22_HUM_VALUE = dht.readHumidity();
  if (isnan(DTH22_TEMP_VALUE)) {
    DTH22_TEMP_VALUE = 0;
  }
  if (isnan(DTH22_HUM_VALUE)) {
    DTH22_HUM_VALUE = 0;
  }
}

void leer_SR04(){
  float duration_us;
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
    // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);
  
  // calculate the distance
  SR04_VALUE = 0.017 * duration_us;
}

// LDR Characteristics
//const float GAMMA = 0.7;
//const float RL10 = 50;
void leer_LDR(){
  LDR_VALUE = analogRead(LDR_PIN); // Read the analog value of the LDR
  //float voltage = LDR_VALUE / 1024. * 5;
  //float resistance = 2000 * voltage / (1 - voltage / 5);
  //LDR_LUX_VALUE = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));
}

void leer_MQ2(){
  MQ2_VALUE = analogRead(MQ2_ANA_PIN);
}


void mostrar_sensores(){
  Serial.print("LUZ = ");
  Serial.println(LDR_VALUE);   // the raw analog reading
  Serial.print("ULTRASONIDO = ");
  Serial.println(SR04_VALUE);   // the raw analog reading
  Serial.print("TEMPERATURA = ");
  Serial.println(DTH22_TEMP_VALUE);   // the raw analog reading
  Serial.print("HUMEDAD = ");
  Serial.println(DTH22_HUM_VALUE);   // the raw analog reading
  Serial.print("Presencia = ");
  Serial.println(PIR_VALUE);   // the raw analog reading
  Serial.print("GAS = ");
  Serial.println(MQ2_VALUE);   // the raw analog 
  Serial.print("Vibración = ");
  Serial.println(VIB_VALUE);   // the raw analog reading
  Serial.print("Micrófono = ");
  Serial.println(MIC_VALUE);   // the raw analog reading
}


void loop() {
 
 if (mode_board == 0){
  leer_LDR();
  leer_SR04();
  leer_DTH22();
  leer_PIR();
  leer_MQ2();
  leer_VIB();
  leer_MIC();
 }

  //float a1 = analogRead(35);
  //Serial.println(a1);  
  //mostrar_sensores();
  //delay(500);

  delay(10);
  
  //desactiva modo inyeccion si pasan mas de 5 minutos sin reactivarlo
  if (temporizador_activo && mode_board == 1) {
      unsigned long tiempo_actual = millis();
      if (tiempo_actual - tiempo_inicio_inyeccion >= TIEMPO_MAX_INYECCION) {
        mode_board = 0;
        temporizador_activo = false;
        Serial.println("Se desactivó modo inyección por timeout");
      }
  }

  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

}