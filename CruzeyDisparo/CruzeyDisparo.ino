// --- Pines ---
const int pinCruce = 2;
const int pinDisparo = 3;

// --- Tiempos para 60Hz ---
// Semi-ciclo = 8333 us. 50% = 4166 us.
const unsigned long retardo_us = 4166; 

// Variables de control de tiempo
unsigned long tiempoCruce = 0;
bool esperandoDisparo = false;
bool disparando = false;
unsigned long tiempoDisparo = 0;
int estadoCruceAnterior = LOW;

void setup() {
  Serial.begin(115200); 
  
  pinMode(pinCruce, INPUT_PULLUP);
  pinMode(pinDisparo, OUTPUT);
  digitalWrite(pinDisparo, LOW);
}

void loop() {
  unsigned long tiempoActual = micros();
  
  // 1. Leer el pin de cruce
  int estadoCruce = digitalRead(pinCruce);

  // Detectar el inicio del pico (flanco de subida)
  if (estadoCruce == HIGH && estadoCruceAnterior == LOW) {
    tiempoCruce = tiempoActual;
    esperandoDisparo = true;
  }
  estadoCruceAnterior = estadoCruce;

  // 2. Control del disparo (Sin usar delays)
  int estadoDisparoGrafica = LOW;

  if (esperandoDisparo) {
    // Si ya pasó el tiempo exacto del 50% (4166 us)
    if (tiempoActual - tiempoCruce >= retardo_us) {
      esperandoDisparo = false;
      disparando = true;
      tiempoDisparo = tiempoActual;
      digitalWrite(pinDisparo, HIGH);
    }
  }

  if (disparando) {
    estadoDisparoGrafica = HIGH;
    
    // Mantenemos el pulso por 400us SOLO para que el Plotter lo alcance a dibujar.
    if (tiempoActual - tiempoDisparo >= 400) { 
      disparando = false;
      digitalWrite(pinDisparo, LOW);
    }
  }

  // 3. Imprimir para el Serial Plotter
  // Formato: Cruce (x1.5 para hacerlo más alto) [espacio] Disparo
  Serial.print(estadoCruce * 1.5); 
  Serial.print(" ");
  Serial.println(estadoDisparoGrafica);
}