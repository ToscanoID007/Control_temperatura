#include "MAX6675.h"

// ==========================================
// PINES Y OBJETOS
// ==========================================
const int thermoCS  = 10;
const int thermoDO  = 12; // MISO
const int thermoCLK = 13;
MAX6675 termopar(thermoCS, thermoDO, thermoCLK);

const int pinCruce = 2;
const int pinDisparo = 3;

// ==========================================
// VARIABLES DE CONTROL PID (Valores por defecto)
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
// ESTADOS DEL SISTEMA (Nuevas)
// ==========================================
bool sistemaActivo = true; // Controla el Start/Stop (A1/A0)
bool modoManual = false;   // Controla Manual/Digital (M1/M0)

// ==========================================
// VARIABLES DE TIEMPO Y FASE CONTROLES AC
// ==========================================
unsigned long tiempoPIDAnterior = 0;
const int intervaloLectura = 250;

unsigned long retardo_us = 8500; // Inicia apagado
unsigned long tiempoCruce = 0;
unsigned long tiempoDisparo = 0;
bool esperandoDisparo = false;
bool disparando = false;
int estadoCruceAnterior = LOW;

// ==========================================
// COLA (BUFFER) PARA EL SERIAL (Con comas para MATLAB)
// ==========================================
unsigned long tiempoSerialAnterior = 0;
const int intervaloSerial = 200; 

float bufferTemp[2];
float bufferSet[2];
float bufferOut[2];
int indiceBuffer = 0; 

// Buffer para la lectura serial no bloqueante
String cadenaEntrada = "";

void setup() {
  Serial.begin(115200); 
  
  pinMode(thermoDO, INPUT_PULLUP);
  pinMode(pinCruce, INPUT_PULLUP);
  pinMode(pinDisparo, OUTPUT);
  digitalWrite(pinDisparo, LOW);

  termopar.begin();
  delay(500); 
}

void loop() {
  unsigned long tiempoActualMicros = micros();
  unsigned long tiempoActualMillis = millis();

  // ==============================================================
  // 0. LECTURA SERIAL NO BLOQUEANTE (Desde MATLAB)
  // ==============================================================
  while (Serial.available() > 0) {
    char caracter = Serial.read();
    if (caracter == '\n' || caracter == '\r') {
      if (cadenaEntrada.length() > 0) {
        procesarComandoMATLAB(cadenaEntrada);
        cadenaEntrada = ""; // Vaciar buffer para el siguiente comando
      }
    } else {
      cadenaEntrada += caracter;
    }
  }

  // ==============================================================
  // 1. TAREA RÁPIDA: CONTROL DE FASE (Microsegundos - Sin Delays)
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
  // 2. TAREA LENTA: TEMPERATURA Y CÁLCULO PID / MANUAL (Milisegundos)
  // ==============================================================
  if (tiempoActualMillis - tiempoPIDAnterior >= intervaloLectura) {
    double dt = (double)(tiempoActualMillis - tiempoPIDAnterior) / 1000.0;
    tiempoPIDAnterior = tiempoActualMillis;

    uint8_t estado = termopar.read();

    if (estado == 0) { 
      float temperatura = termopar.getCelsius();

      if (!sistemaActivo) {
        // Si el sistema está en STOP, apagado inmediato
        salidaPID = 0.0;
      } 
      else if (modoManual) {
        // MODO MANUAL: El setpoint actúa directamente como % de potencia (0-100)
        salidaPID = setpoint; 
        if (salidaPID > 100.0) salidaPID = 100.0;
        if (salidaPID < 0.0)   salidaPID = 0.0;
      } 
      else {
        // MODO DIGITAL / AUTOMÁTICO: Cálculo de PID estándar
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

      // --- LLENADO DE LA COLA (BUFFER) ---
      if (indiceBuffer < 2) {
        bufferTemp[indiceBuffer] = temperatura;
        bufferSet[indiceBuffer]  = setpoint;
        bufferOut[indiceBuffer]  = salidaPID;
        indiceBuffer++;
      }
      
    } else {
      // Seguridad
      retardo_us = 8500; 
      salidaPID = 0;
    }
  }

  // ==============================================================
  // 3. TAREA DE DEBUG: VACIAR COLA CON SEPARACIÓN POR COMAS
  // ==============================================================
  if (tiempoActualMillis - tiempoSerialAnterior >= intervaloSerial) {
    tiempoSerialAnterior = tiempoActualMillis;

    for (int i = 0; i < indiceBuffer; i++) {
      Serial.print(bufferTemp[i]); 
      Serial.print(",");
      Serial.print(bufferSet[i]);  
      Serial.print(",");
      Serial.println(bufferOut[i]); // MATLAB recibe "Temp,Setpoint,Actuador\n"
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
    case 'A': // Arranque / Paro ("A1" o "A0")
      sistemaActivo = ((int)valor == 1);
      break;
    case 'M': // Manual / Digital ("M1" o "M0")
      modoManual = ((int)valor == 1);
      break;
    case 'S': // Setpoint (Temperatura en Digital o % en Manual)
      setpoint = valor;
      break;
    case 'P': // Kp
      kp = valor;
      break;
    case 'I': // Ki
      ki = valor;
      break;
    case 'D': // Kd
      kd = valor;
      break;
  }
}