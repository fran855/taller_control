// La stdint.h define los ancho de datos como el uint16_t
#include <stdint.h>

// Definimos los pines a utilizar
#define PIN_PWM 9 //OC1A
#define PIN_POTE A0
#define MAXIMO_POTE 682
#define MINIMO 1200
#define MAXIMO 4900
#define TOP 39999
#define FRECUENCIA_LECTURA 100
uint16_t periodo_lectura;
uint16_t tiempo_inicial = 0;
uint16_t tiempo_final = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  config_50_hz();
  periodo_lectura = 1e3/FRECUENCIA_LECTURA;
}

void loop() {
  // Utilizamos millis() en lugar de micros() porque esta última llega hasta 65536 (2^16) y necesitaríamos del orden de los 10^6 para 10 Hz
  tiempo_inicial = millis();
  int valor_pote = analogRead(PIN_POTE);
  OCR1A = procesar_valor_pote(valor_pote);
  tiempo_final = millis();
  delay(periodo_lectura - (tiempo_final - tiempo_inicial));
}


void config_50_hz(){ 
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = TOP;
}

uint16_t procesar_valor_pote(int valor_pote){
  if(valor_pote > MAXIMO_POTE){
    Serial.println("Ángulo superado. Recortado a 180°");
    return (uint16_t) MAXIMO;
  }
  else{
    uint16_t angulo = map(valor_pote, 0, MAXIMO_POTE, 0, 180);
    Serial.print("Ángulo: ");
    Serial.println(angulo);
    return map(valor_pote, 0, MAXIMO_POTE, MINIMO, MAXIMO);
  }
}
