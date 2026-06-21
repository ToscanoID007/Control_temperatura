#include "MAX6675.h"

// --- Pines de conexión ---
const int thermoCS  = 10; // SELECT (Chip Select)
const int thermoDO  = 12; // MISO (Signal Out) - ¡Ponle su Pull-Up a 5V!
const int thermoCLK = 13; // CLOCK

// Constructor SPI por Software (SELECT, MISO, CLOCK)
MAX6675 termopar(thermoCS, thermoDO, thermoCLK);

void setup() {
  // Alta velocidad para que la gráfica fluya chido
  Serial.begin(115200);

  // Inicializamos la librería obligatoriamente
  termopar.begin();
  
  delay(500); // Tiempo para que el chip estabilice
}

void loop() {
  // 1. Disparamos la lectura, esto devuelve el código de estado
  uint8_t estado = termopar.read();

  // 2. Verificamos si la lectura fue exitosa (STATUS_OK == 0)
  if (estado == 0) { 
    // Extraemos e imprimimos para el Serial Plotter
    float temperatura = termopar.getCelsius();
    Serial.println(temperatura);
  } 
  else {
    // Si hay error, lo reportamos. El Plotter bajará la gráfica a 0 
    // o simplemente se pausará si mandas texto en lugar de números.
    if (estado == 4) {
      Serial.println("Error: Cortocircuito a VCC en el termopar");
    } 
    else if (estado == 129) {
      Serial.println("Error: Sin comunicacion (Revisa cables y el Pull-up en MISO)");
    } 
    else {
      Serial.println("Error de lectura desconocido");
    }
  }

  // Intervalo de lectura (500ms es excelente para el Plotter)
  delay(500);
}