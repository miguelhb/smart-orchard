/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.

const char* ssid = "GISAI NET";
const char* password = "GisaiEtsisiNet19";
const char* mqtt_server = "10.30.1.250";

//const char* ssid = "MOVISTAR_8625";
//const char* password = "ilnmNhwnjVXMJ2ePod64";

//moisture sensor
  int sensorPin = A0;
  int sensorValue = 0;

  volatile int NumPulsos; //variable para la cantidad de pulsos recibidos
int PinSensor = D1;    //Sensor conectado en el pin 2
float factor_conversion=7.5; //para convertir de frecuencia a caudal


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

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

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("casa/despacho/flujo", "Enviando el primer mensaje");
      // ... and resubscribe
      client.subscribe("casa/despacho/luz");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void ICACHE_RAM_ATTR ContarPulsos ();

//---Función que se ejecuta en interrupción---------------
void ContarPulsos (){ 
  NumPulsos++;  //incrementamos la variable de pulsos
 // Serial.print("Interrupcion");
} 

//---Función para obtener frecuencia de los pulsos--------
int ObtenerFrecuencia() {
  int frecuencia;
  NumPulsos = 0;   //Ponemos a 0 el número de pulsos
  interrupts();    //Habilitamos las interrupciones
  delay(1000);   //muestra de 1 segundo
  noInterrupts(); //Desabilitamos las interrupciones
  frecuencia=NumPulsos; //Hz(pulsos por segundo)
  return frecuencia;
}


void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(PinSensor, INPUT); 
  attachInterrupt(digitalPinToInterrupt(5),ContarPulsos,RISING); //(Interrupcion 0(Pin2),funcion,Flanco de subida)
}


void loop() {

    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

      if (!client.connected()) {
        reconnect();
      }
      if(!client.loop()) client.connect(clientId.c_str());

    long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;

  float frecuencia=ObtenerFrecuencia(); //obtenemos la Frecuencia de los pulsos en Hz
  float caudal_L_m=frecuencia/factor_conversion; //calculamos el caudal en L/m
  float caudal_L_h=caudal_L_m*60; //calculamos el caudal en L/h

  //-----Enviamos por el puerto serie---------------
  Serial.print ("FrecuenciaPulsos: "); 
  Serial.print (frecuencia,0); 
  Serial.print ("Hz\tCaudal: "); 
  Serial.print (caudal_L_m,3); 
  Serial.print (" L/m\t"); 
   Serial.print (caudal_L_h,3); 
  Serial.println ("L/h"); 
      
      // read the value from the sensor:
   //   sensorValue = analogRead(sensorPin);
   //   Serial.print("Moisture = " );
   //   Serial.println(sensorValue);
    //  snprintf(aux,50, "hello world #%ld", value);
   //    char b[2];
   //    String auxString=String(sensorValue);
   //    char aux[10];
   //    auxString.toCharArray(aux, 10);

   //   client.publish("casa/despacho/flujo", aux); //msg
  }
  delay(10000);

//      char* aux;
//    snprintf (msg, 50, "hello world #%ld", value);
//    snprintf(aux,50, "hello world #%ld", value);
 //   Serial.print("Publish message: ");
 //   Serial.println(sensorValue); //msg
//    client.publish("casa/despacho/temperatura", aux); //msg
  
}
