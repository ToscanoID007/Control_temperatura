#include "MAX6675.h"

// --- Pines de Hardware ---
const int pinCruce = 2;    
const int pinDisparo = 3;  
const int thermoCS  = 10;  
const int thermoDO  = 12;  
const int thermoCLK = 13;  

// --- Instancia del Termopar ---
MAX6675 termopar(thermoCS, thermoDO, thermoCLK);

// --- Variables de Control de Estado ---
bool sistemaActivo = false; // Arranca en FALSE (PARO) por seguridad
bool modoManual = false;    // false = Automático (PID), true = Manual

// --- Variables del PID ---
float setpoint = 40.0;    
float kp = 2.0, ki = 0.0, kd = 0.0; 
float temperatura = 0.0;
float potencia = 0.0;     
float potenciaManual = 0.0;
unsigned long retardo_us = 8333; 

// Variables internas PID
float errorAnterior = 0.0;
float integral = 0.0;
unsigned long ultimoTiempoPID = 0;
const unsigned long intervaloPID = 250; 

volatile bool cruceDetectado = false;

void setup() {
  Serial.begin(115200); // Velocidad alta para MATLAB
  
  termopar.begin();
  pinMode(pinCruce, INPUT_PULLUP);
  pinMode(pinDisparo, OUTPUT);
  digitalWrite(pinDisparo, LOW);
  
  attachInterrupt(digitalPinToInterrupt(pinCruce), interrupcionCruce, RISING);
  delay(500); 
}

void interrupcionCruce() {
  cruceDetectado = true;
}

void loop() {
  unsigned long tiempoActual = millis();

  // ==========================================
  // 1. CONTROL DE DISPARO DEL TRIAC
  // ==========================================
  if (cruceDetectado) {
    // Solo dispara si el sistema está en ARRANQUE y la potencia es mayor a 0
    if (sistemaActivo && potencia > 0.5) {
      if (potencia < 99.5) {
        delayMicroseconds(retardo_us);
      }
      digitalWrite(pinDisparo, HIGH);
      delayMicroseconds(15); 
      digitalWrite(pinDisparo, LOW);
    }
    cruceDetectado = false;
  }

  // ==========================================
  // 2. EJECUCIÓN DEL PID Y TELEMETRÍA (Cada 250ms)
  // ==========================================
  if (tiempoActual - ultimoTiempoPID >= intervaloPID) {
    float dt = (tiempoActual - ultimoTiempoPID) / 1000.0; 
    ultimoTiempoPID = tiempoActual;

    // Leer sensor de forma segura
    if (termopar.read() == 0) {
      temperatura = termopar.getCelsius();
    }

    // Calcular acción de control basándose en el estado de Arranque/Paro
    if (sistemaActivo) {
      if (!modoManual) {
        // --- MODO AUTOMÁTICO (PID) ---
        float error = setpoint - temperatura;
        float proporcional = kp * error;
        
        integral += ki * error * dt;
        integral = constrain(integral, 0, 100); // Anti-windup
        
        float derivativa = kd * (error - errorAnterior) / dt;
        errorAnterior = error;

        potencia = proporcional + integral + derivativa;
        potencia = constrain(potencia, 0, 100);
      } else {
        // --- MODO MANUAL ---
        potencia = potenciaManual;
        integral = 0;
        errorAnterior = 0;
      }
      
      // Calcular retraso para 60Hz
      retardo_us = 8200 - (potencia * 81.0); 
    } 
    else {
      // --- MODO PARO ---
      potencia = 0;
      integral = 0;
      errorAnterior = 0;
    }

    // --- ENVIAR DATOS A MATLAB ---
    // Formato: Temp,Setpoint,Potencia
    Serial.print(temperatura);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(potencia);
  }

  // ==========================================
  // 3. LECTURA DE COMANDOS DESDE MATLAB
  // ==========================================
  if (Serial.available() > 0) {
    char comando = Serial.read(); 
    float valor = Serial.parseFloat(); 
    
    while(Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();
    }

    switch (comando) {
      case 'A': // Arranque / Paro (1 = Arrancar, 0 = Parar)
        sistemaActivo = (valor > 0.5);
        break;
      case 'S': // Setpoint
        setpoint = constrain(valor, 0, 150); 
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
      case 'M': // Modo (1 = Manual, 0 = Auto)
        modoManual = (valor > 0.5);
        break;
      case 'W': // Potencia Manual
        potenciaManual = constrain(valor, 0, 100);
        break;
    }
  }
}