/*
// Include Emon Library
#include "EmonLib.h"
 
// Crear una instancia EnergyMonitor
EnergyMonitor energyMonitor;
 
// Voltaje de nuestra red eléctrica
float voltajeRed = 220.0;
 
void setup()
{
  Serial.begin(9600);
 
  // Iniciamos la clase indicando
  // Número de pin: donde tenemos conectado el SCT-013
  // Valor de calibración: valor obtenido de la calibración teórica
  energyMonitor.current(0, 5);
}
 
void loop()
{
  // Obtenemos el valor de la corriente eficaz
  // Pasamos el número de muestras que queremos tomar
  double Irms = energyMonitor.calcIrms(1484);
 
  // Calculamos la potencia aparente
  double potencia =  Irms * voltajeRed;
 
  // Mostramos la información por el monitor serie
  Serial.print("Potencia = ");
  Serial.print(potencia);
  Serial.print("    Irms = ");
  Serial.println(Irms);
}

*/
/*
 * Medida de consumo eléctrico con sensor inducción SCT-013
 * Con cambio de escala inicial y calculo de factor de potencia
 * MOOC "Introducción al diseño de Sistemas Domóticos: Monitorización y Control del Consumo Energético"
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "GISAI NET";
const char* password = "GisaiEtsisiNet19";
const char* mqtt_server = "10.30.1.250";

//const char* ssid = "MOVISTAR_8625";
//const char* password = "ilnmNhwnjVXMJ2ePod64";

//moisture sensor
  int sensorPin = A0;
  int sensorValue = 0;
  
//Factor de escala para hallar el valor de pico de corriente de entrada
//a partir del valor medido por el Arduino 
float FACTOR = 6.3;
        
//Valores máximos y mínimos de la escala, 
//Vmax es el fondo alcanzado para cada escala
float VMIN = 0;
float VMAX = 3.3;

//Ciclos que adquirimos para hacer la medida
float numCiclosLec = 50;

//Valor efectivo de la tensión en el sistema eléctrico
const float voltageRMS = 230;
 
const float ADCV = 3.3;  //Vcc (Arduino alimnetado por USB, 5V)

boolean lect_continua;
boolean waitingPeriodo;

//Factor de potencia introducido por el usuario
float factorPotencia;

//Límites para detectar un nuevo periodo
int minLimit;
int maxLimit;

// Definición de una estructura
struct Medidas {
    float corriente;
    float potencia;
};

/*
 ****************************************************************** 
 */


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


/*
**********************************************************************
*/

/*
 * Configuración
 */
void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
   //Comunicación con el ordenador
   //Serial.begin(9600);
}

void printMeasure(String prefix, float value, String postfix)
{
   Serial.print(prefix);
   Serial.print(value, 3);
   Serial.println(postfix);
}

/**
 * Función para calcular los límites 
 * de un semiperiodo
 */
void calculaLimites () {
  int valorMax = 0;
  int voltage;
  long tiempo = millis();

   while (millis() - tiempo < 20*numCiclosLec)
   {
      voltage = analogRead(A0);
      if (voltage > valorMax) {
        valorMax = voltage;
      }
      delay(1);
   }
 
   minLimit = (int)(0.2*valorMax);
   maxLimit = (int)(0.8*valorMax);
}

 /**
  * Buble principal
  */
void loop()
{
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

  //-----Enviamos por el puerto serie---------------
   Medidas medidas = getCorriente();
   float currentRMS = medidas.corriente;
   float powerMed = medidas.potencia;
   float power = voltageRMS* currentRMS;
 
   printMeasure("Irms: ", currentRMS, "A ,");
   printMeasure("Potencia aparente: ", power, "W");
   printMeasure("Potencia activa según fdp usuario: ", factorPotencia*power, "W");
   printMeasure("Potencia activa: ", powerMed, "W");   
   printMeasure("Factor de potencia calculado: ", powerMed/power, " ");
   
      // read the value from the sensor:
   //   sensorValue = analogRead(sensorPin);
   //   Serial.print("Moisture = " );
   //   Serial.println(sensorValue);
    //  snprintf(aux,50, "hello world #%ld", value);
   //    char b[2];
   //    String auxString=String(sensorValue);
   //    char aux[10];
   //    auxString.toCharArray(aux, 10);
       String auxString0=String(powerMed);
       char aux0[10];
       auxString0.toCharArray(aux0, 10);
       String auxString1=String(power);
       char aux1[10];
       auxString1.toCharArray(aux1, 10);


      client.publish("casa/despacho/luzActiva", aux0); //msg
      client.publish("casa/despacho/luzAparente", aux1); //msg
  }
  

    //Hacemos una medida de consumo cada 20 segundos
    delay(2000);

}

/**
 * Función para calcular el valor efectivo de la corriente
 */
Medidas getCorriente()
{
   float voltage;
   float corriente;
   Medidas medidas;
   float sumPot = 0;
   float sum = 0;
   int contador_periodos = 0;
   int counter = 0;
   
   calculaLimites();
  // while (contador_periodos <= numCiclosLec*2)
   while(counter<500)
   {
      float lectura = analogRead(A0);
      voltage =  lectura * ADCV / 1023.0;  
    //  if (lectura > minLimit && lectura < maxLimit) {
    //    if (waitingPeriodo) {
    //     contador_periodos = contador_periodos+1;
    //     waitingPeriodo = false;
    //    }
    //  } else {
    //     waitingPeriodo = true;
    //  }
      corriente = fmap(voltage, VMIN, VMAX, 0, FACTOR);  
      counter = counter+1;
      sum += sq(corriente);
      sumPot += voltageRMS*corriente;
      delay(1);
   }
   Serial.println(counter);
   medidas.corriente = sqrt(sum / counter);
   medidas.potencia = sumPot/counter;
   return(medidas);
}

 
// cambio de escala entre floats
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
