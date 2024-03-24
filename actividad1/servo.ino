// La stdint.h define los ancho de datos como el uint16_t
#include <stdint.h>

// Definimos los pines a utilizar
const int PIN_PWM = 9; //OC1A
const int PIN_POTE = A0;
const int MAXIMO_POTE = 682;
const int MINIMO = 1200;
const int MAXIMO = 4900;
const int TOP = 39999;
const unsigned int PERIODO_PWM = 200;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  config_50_hz();
  delay(100);

  int valor_pote = analogRead(PIN_POTE);

  OCR1A = procesar_valor_pote(valor_pote);
  
  delay(1000);
}

void loop() {
  unsigned int tiempo_inicial = millis();

  int valor_pote = analogRead(PIN_POTE);

  OCR1A = procesar_valor_pote(valor_pote);

  unsigned int tiempo_final = millis();

  
  delay(PERIODO_PWM - (tiempo_final - tiempo_inicial));
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
