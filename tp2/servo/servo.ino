// La stdint.h define los ancho de datos como el uint16_t
#include <stdint.h>

// Definimos los pines a utilizar
#define PIN_PWM 9 //OC1A
#define PIN_POTE A0
#define MINIMO_POTE 10
#define MAXIMO_POTE 712
#define MINIMO 1350
#define MAXIMO 5100
#define TOP 39999
#define FRECUENCIA_LECTURA 100

uint16_t offset_angulo = 5;
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
  int angle;
  while (Serial.available() > 0) {
    angle = Serial.read();
  }
  OCR1A = angulo_a_servo(angle);

  int valor_pote = analogRead(PIN_POTE);
  int angulo = convertir_a_angulo(valor_pote);
  
  //matlab_send(angulo, 0, 0);

  matlab_send(angulo, tiempo_inicial, tiempo_final);

  tiempo_final = millis();

  delay(periodo_lectura - (tiempo_final - tiempo_inicial));
}


void config_50_hz(){ 
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = TOP;
}


int procesar_valor_pote(int valor_pote){
    //int angulo = map(valor_pote, MINIMO_POTE, MAXIMO_POTE, 0, 180);
    //Serial.print("Ángulo: ");
    //Serial.println(angulo);
    return map(valor_pote, MINIMO_POTE, MAXIMO_POTE, MINIMO, MAXIMO);
}

int convertir_a_angulo(int valor_pote){
  return map(valor_pote, MINIMO_POTE, MAXIMO_POTE, 0, 180);
}

int angulo_a_servo(int angulo){
  return map(angulo, 0, 180, MINIMO, MAXIMO);
}

void matlab_send(float dato1, float dato2, float dato3){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
}
