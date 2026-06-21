#include "MAX6675.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==========================================
// CONFIGURACIÓN DE PANTALLA OLED
// ==========================================
#define ANCHO_PANTALLA 128 // Ancho en píxeles
#define ALTO_PANTALLA 64   // Alto en píxeles
#define OLED_RESET -1      // Pin de reset (compartido con Arduino)
#define DIRECCION_I2C 0x3C // Dirección I2C estándar para 0.96"

Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, OLED_RESET);

// ==========================================
// PINES Y OBJETOS TERMOPAR
// ==========================================
const int thermoCS  = 10;
const int thermoDO  = 12; // MISO
const int thermoCLK = 13;
MAX6675 termopar(thermoCS, thermoDO, thermoCLK);

const int pinCruce = 2;
const int pinDisparo = 3;

// ==========================================
// VARIABLES DE CONTROL PID
// ==========================================
double kp = 2.5;         
double ki = 0.1;
double kd = 0.8;
double setpoint = 50.0;  

double error = 0.0;
double errorAnterior = 0.0;
double integral = 0.0;
double derivada = 0.0;
double salidaPID = 0.0;  

// ==========================================
// ESTADOS DEL SISTEMA
// ==========================================
bool sistemaActivo = true; // Controla el Start/Stop (A1/A0)
bool modoManual = false;   // Controla Manual/Digital (M1/M0)

// ==========================================
// VARIABLES DE TIEMPO Y FASE AC
// ==========================================
unsigned long tiempoPIDAnterior = 0;
const int intervaloLectura = 250;

unsigned long tiempoPantallaAnterior = 0;
const int intervaloPantalla = 500; // Actualiza OLED cada medio segundo

unsigned long retardo_us = 8500; // Inicia apagado
unsigned long tiempoCruce = 0;
unsigned long tiempoDisparo = 0;
bool esperandoDisparo = false;
bool disparando = false;
int estadoCruceAnterior = LOW;

// ==========================================
// COLA (BUFFER) PARA EL SERIAL
// ==========================================
unsigned long tiempoSerialAnterior = 0;
const int intervaloSerial = 200; 

float bufferTemp[2];
float bufferSet[2];
float bufferOut[2];
int indiceBuffer = 0; 

// Variables globales temporales para mostrar en pantalla
float tempActualGlobal = 0.0;

// Buffer para la lectura serial
String cadenaEntrada = "";

void setup() {
  Serial.begin(115200); 
  
  pinMode(thermoDO, INPUT_PULLUP);
  pinMode(pinCruce, INPUT_PULLUP);
  pinMode(pinDisparo, OUTPUT);
  digitalWrite(pinDisparo, LOW);

  termopar.begin();
  
  // Iniciar pantalla OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, DIRECCION_I2C)) {
    Serial.println(F("Fallo OLED")); // Falla silenciosa para no detener el sistema
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.display();
  }

  delay(500); 
}

void loop() {
  unsigned long tiempoActualMicros = micros();
  unsigned long tiempoActualMillis = millis();

  // ==============================================================
  // 0. LECTURA SERIAL NO BLOQUEANTE (Desde Python/MATLAB)
  // ==============================================================
  while (Serial.available() > 0) {
    char caracter = Serial.read();
    if (caracter == '\n' || caracter == '\r') {
      if (cadenaEntrada.length() > 0) {
        procesarComandoMATLAB(cadenaEntrada);
        cadenaEntrada = "";
      }
    } else {
      cadenaEntrada += caracter;
    }
  }

  // ==============================================================
  // 1. TAREA RÁPIDA: CONTROL DE FASE (Microsegundos)
  // ==============================================================
  int estadoCruce = digitalRead(pinCruce);

  if (estadoCruce == HIGH && estadoCruceAnterior == LOW) {
    tiempoCruce = tiempoActualMicros;
    esperandoDisparo = true;
  }
  estadoCruceAnterior = estadoCruce;

  if (esperandoDisparo) {
    if (tiempoActualMicros - tiempoCruce >= retardo_us && retardo_us <= 8333) {
      esperandoDisparo = false;
      disparando = true;
      tiempoDisparo = tiempoActualMicros;
      digitalWrite(pinDisparo, HIGH);
    }
  }

  if (disparando) {
    if (tiempoActualMicros - tiempoDisparo >= 400) { 
      disparando = false;
      digitalWrite(pinDisparo, LOW);
    }
  }

  // ==============================================================
  // 2. TAREA MEDIA: TEMPERATURA Y PID (Milisegundos)
  // ==============================================================
  if (tiempoActualMillis - tiempoPIDAnterior >= intervaloLectura) {
    double dt = (double)(tiempoActualMillis - tiempoPIDAnterior) / 1000.0;
    tiempoPIDAnterior = tiempoActualMillis;

    uint8_t estado = termopar.read();

    if (estado == 0) { 
      float temperatura = termopar.getCelsius();
      tempActualGlobal = temperatura; // Guardar para la pantalla OLED

      if (!sistemaActivo) {
        salidaPID = 0.0;
      } 
      else if (modoManual) {
        salidaPID = setpoint; 
        if (salidaPID > 100.0) salidaPID = 100.0;
        if (salidaPID < 0.0)   salidaPID = 0.0;
      } 
      else {
        error = setpoint - temperatura;
        integral = integral + (error * dt);
        
        // Anti-windup
        if (integral > 1000.0) integral = 1000.0;
        if (integral < 0.0)   integral = 0.0;

        derivada = (error - errorAnterior) / dt;
        salidaPID = (kp * error) + (ki * integral) + (kd * derivada);

        if (salidaPID > 100.0) salidaPID = 100.0;
        if (salidaPID < 0.0)   salidaPID = 0.0;

        if (error < -0.5) { 
          salidaPID = 0.0;
        }
        errorAnterior = error;
      }

      // --- MAPEAR PID AL RETARDO DEL TRIAC ---
      if (salidaPID == 0) {
        retardo_us = 8500; 
      } else {
        retardo_us = map((long)salidaPID, 1, 100, 8000, 200);
      }

      // --- LLENADO DE LA COLA ---
      if (indiceBuffer < 2) {
        bufferTemp[indiceBuffer] = temperatura;
        bufferSet[indiceBuffer]  = setpoint;
        bufferOut[indiceBuffer]  = salidaPID;
        indiceBuffer++;
      }
      
    } else {
      retardo_us = 8500; 
      salidaPID = 0;
    }
  }

  // ==============================================================
  // 3. TAREA DE PANTALLA OLED (Cada 500 ms para no bloquear el PID)
  // ==============================================================
  if (tiempoActualMillis - tiempoPantallaAnterior >= intervaloPantalla) {
    tiempoPantallaAnterior = tiempoActualMillis;
    actualizarPantalla(tempActualGlobal, setpoint);
  }

  // ==============================================================
  // 4. TAREA DE DEBUG: VACIAR COLA AL SERIAL
  // ==============================================================
  if (tiempoActualMillis - tiempoSerialAnterior >= intervaloSerial) {
    tiempoSerialAnterior = tiempoActualMillis;

    for (int i = 0; i < indiceBuffer; i++) {
      Serial.print(bufferTemp[i]); 
      Serial.print(",");
      Serial.print(bufferSet[i]);  
      Serial.print(",");
      Serial.println(bufferOut[i]); 
    }
    indiceBuffer = 0; 
  }
}

// ==============================================================
// FUNCIÓN AUXILIAR: PROCESAR COMANDOS RECIBIDOS
// ==============================================================
void procesarComandoMATLAB(String cmd) {
  char tipoComando = cmd.charAt(0);
  String valorStr = cmd.substring(1);
  float valor = valorStr.toFloat();

  switch (tipoComando) {
    case 'A': sistemaActivo = ((int)valor == 1); break;
    case 'M': modoManual = ((int)valor == 1); break;
    case 'S': setpoint = valor; break;
    case 'P': kp = valor; break;
    case 'I': ki = valor; break;
    case 'D': kd = valor; break;
  }
}

// ==============================================================
// FUNCIÓN AUXILIAR: ACTUALIZAR PANTALLA OLED
// ==============================================================
void actualizarPantalla(float temp, float sp) {
  display.clearDisplay();

  // --- ZONA AMARILLA SUPERIOR ---
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(sistemaActivo ? "ESTADO: ON" : "ESTADO: OFF");

  display.setCursor(70, 0);
  display.print(modoManual ? "MODO: MAN" : "MODO: AUT");

  display.drawLine(0, 14, 128, 14, SSD1306_WHITE); // Línea divisoria

  // --- ZONA AZUL INFERIOR ---
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Temp Actual:");

  // Temperatura en texto grande
  display.setTextSize(2);
  display.setCursor(0, 35);
  display.print(temp, 1); 
  display.print(" C");

  // Setpoint en texto pequeño
  display.setTextSize(1);
  display.setCursor(80, 42);
  display.print("SP:");
  display.print(sp, 0);

  display.display();
}