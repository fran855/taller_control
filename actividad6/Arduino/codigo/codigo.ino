// La stdint.h define los ancho de datos como el uint16_t
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h> 
#include <stdint.h>

Adafruit_MPU6050 mpu;
Servo servo;

// Definimos los pines a utilizar
#define PIN_PWM 9 //OC1A
#define PIN_POTE A0
#define MINIMO_POTE 10
#define MAXIMO_POTE 712
#define MINIMO 1350
#define MAXIMO 5100
#define TOP 39999
#define FRECUENCIA_LECTURA 100

int LIMITE_ANGULO_SUPERIOR = 135;
int LIMITE_ANGULO_INFERIOR = 45;

uint16_t offset_angulo = 5;
uint16_t periodo_lectura;
uint16_t tiempo_inicial = 0;
uint16_t tiempo_final = 0;

float offset_giro_x = -0.04;
float offset_giro_y = 0.03;
float offset_giro_z = 0;

float offset_accel_x = 1.13;
float offset_accel_y = 0.39;
float offset_accel_z = -0.75;

float w_giro_x = 0;
float angulo_gir_x = 0;
float angulo_accel_x = 0;
float angulo_x = 0;

float alpha = 0.98;

// ACTIVIDAD 6
float theta_k = 0;
float theta_k_1 = 0;
float w_k = 0;
float w_k_1 = 0;
float l_1 = 0.8738;
float l_2 = 17.0939;
float a_11 = 1;
float a_12 = 0.01;
float a_21 = -0.6125;
float a_22 = 0.9738;

void setup() {
  Serial.begin(115200);

  // Config IMU
  // Try to initialize!

	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");
	
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // set accelerometer range to +-8G
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // set gyro range to +- 500 deg/s
	mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);   // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  
  pinMode(PIN_PWM, OUTPUT);
  config_50_hz();
  periodo_lectura = 1e6/FRECUENCIA_LECTURA;
}

void loop() {
  // Utilizamos millis() en lugar de micros() porque esta última llega hasta 65536 (2^16) y necesitaríamos del orden de los 10^6 para 10 Hz
  tiempo_inicial = micros();
  int angulo;
  int angulo_a_mover;

  while (Serial.available() > 0) {
    angulo = Serial.read();
  }

  if(angulo > LIMITE_ANGULO_SUPERIOR)
    angulo_a_mover = LIMITE_ANGULO_SUPERIOR;
  else if(angulo < LIMITE_ANGULO_INFERIOR)
    angulo_a_mover = LIMITE_ANGULO_INFERIOR;
  else
    angulo_a_mover = angulo;

  OCR1A = angulo_a_servo(angulo_a_mover);
  
  int valor_pote = analogRead(PIN_POTE);
  int angulo_pote = convertir_a_angulo(valor_pote);
  
  obtener_angulo_giroscopo_x();
  obtener_angulo_accel_x();
  
  angulo_x = alpha*angulo_gir_x + (1-alpha)*angulo_accel_x;

  theta_k_1 = a_11 * theta_k + a_12 * w_k + l_1 * (angulo_x - theta_k);
  w_k_1 = a_21 * theta_k + a_22 * w_k + l_2 * (angulo_x - theta_k);
  theta_k = theta_k_1;
  w_k = w_k_1;

  matlab_send(angulo_x, theta_k_1, w_giro_x, w_k_1);

  tiempo_final = micros();
  delayMicroseconds(periodo_lectura - (tiempo_final - tiempo_inicial));
}


void config_50_hz(){ 
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = TOP;
}


int procesar_valor_pote(int valor_pote){
    return map(valor_pote, MINIMO_POTE, MAXIMO_POTE, MINIMO, MAXIMO);
}

int convertir_a_angulo(int valor_pote){
  return map(valor_pote, MINIMO_POTE, MAXIMO_POTE, 0, 180);
}

int angulo_a_servo(int angulo){
  return map(angulo, 0, 180, MINIMO, MAXIMO);
}

void matlab_send(float dato1, float dato2, float dato3, float dato4){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
  b = (byte *) &dato4;
  Serial.write(b,4);
}

void obtener_angulo_giroscopo_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  w_giro_x = (g.gyro.x - offset_giro_x) * 180 / M_PI;  
  angulo_gir_x = angulo_x + (g.gyro.x-offset_giro_x) * (periodo_lectura/1e6) * 180 / M_PI; 
}

void obtener_angulo_accel_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_accel_x  = atan2(a.acceleration.y-offset_accel_y, a.acceleration.z-offset_accel_z) * 180 / M_PI;
}