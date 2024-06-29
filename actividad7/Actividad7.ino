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

sensors_event_t a, g, temp;
float ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;

uint16_t periodo_lectura;
uint16_t tiempo_inicial = 0;
uint16_t tiempo_final = 0;

float w_giro_x = 0.0;
float angulo_gir_x = 0;
float angulo_accel_x = 0;
float angulo_x = 0;
float angulo_phi = 0;
float angulo_theta = 0;

float referencia_phi = 90;

float alpha = 0.98;
float T = 0.01;

float kp2 = 0.6;
float ki2 = 0.00;
float kp = 0.6;
float ki = 0.00;
float error_phi_actual = 0.0;
float error_phi_anterior = 0.0;
float error_theta_actual = 0.0;
float error_theta_anterior = 0.0;
float u_phi = 0.0;
float u_theta = 0.0;
float i_actual = 0.0;
float i_anterior = 0.0;
float i_phi_actual = 0.0;
float i_theta_anterior = 0.0;
float i_theta_actual = 0.0;
float i_phi_anterior = 0.0;

void setup() {
  Serial.begin(115200);

  // Config IMU
  // Try to initialize!
  while(!mpu.begin()){
		Serial.println("Failed to find MPU6050 chip");
		delay(10);
	}
	Serial.println("MPU6050 Found!");
	
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // set accelerometer range to +-8G
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // set gyro range to +- 500 deg/s
	mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);   // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  
  pinMode(PIN_PWM, OUTPUT);
  config_50_hz();
  periodo_lectura = 1e6/FRECUENCIA_LECTURA;
  OCR1A = angulo_a_servo(60);
  delay(2000);
  calibracion();
  delay(500);
}

void loop() {
  // Utilizamos millis() en lugar de micros() porque esta última llega hasta 65536 (2^16) y necesitaríamos del orden de los 10^6 para 10 Hz
  tiempo_inicial = micros();
  float angulo_a_mover;

  while (Serial.available() > 0) {
    referencia_phi = Serial.read();
  }

  obtener_angulo_giroscopo_x();
  obtener_angulo_accel_x();
  
  angulo_phi = convertir_a_angulo(analogRead(PIN_POTE));
  angulo_x = alpha * angulo_gir_x + (1 - alpha) * angulo_accel_x;
  angulo_theta = angulo_x;

  error_phi_actual = referencia_phi - angulo_phi;
  i_phi_actual = i_phi_anterior + T/2 * error_phi_actual + T/2 * error_phi_anterior;

  u_theta = kp2 * error_phi_actual + ki2 * i_phi_actual;

  error_theta_actual = -(u_theta - angulo_theta);

  i_theta_actual = i_theta_anterior + T/2 * error_theta_actual + T/2 * error_theta_anterior;

  u_phi = kp * error_theta_actual + ki * i_theta_actual;

  angulo_a_mover = u_phi + referencia_phi;

  error_phi_anterior = error_phi_actual;
  error_theta_anterior = error_theta_actual;
  i_theta_anterior = i_theta_actual;
  i_phi_anterior = i_phi_actual;

  if(angulo_a_mover > LIMITE_ANGULO_SUPERIOR)
    angulo_a_mover = LIMITE_ANGULO_SUPERIOR;
  if(angulo_a_mover < LIMITE_ANGULO_INFERIOR)
    angulo_a_mover = LIMITE_ANGULO_INFERIOR;

  OCR1A = angulo_a_servo(angulo_a_mover);

  matlab_send(angulo_x, angulo_phi, 0, 0);

  tiempo_final = micros();
  int diferencia = periodo_lectura - (tiempo_final - tiempo_inicial);
  if(diferencia > 0)
    delayMicroseconds(diferencia);

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

int angulo_a_servo(float angulo){
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
  
  angulo_gir_x = angulo_x + (g.gyro.x-gx_offset) * (periodo_lectura/1e6) * 180 / M_PI; 
}

void obtener_angulo_accel_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  w_giro_x = (g.gyro.x-gx_offset) * 180 / M_PI; 
  angulo_accel_x  = atan2(a.acceleration.y-ay_offset, a.acceleration.z-az_offset) * 180 / M_PI;
}

void imprimir_parametros(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("ACELERACION X: ");
  Serial.print(a.acceleration.x - ax_offset);
  Serial.print(" | ");
  Serial.print("ACELERACION Y: ");
  Serial.print(a.acceleration.y - ay_offset);
  Serial.print(" | ");
  Serial.print("ACELERACION Z: ");
  Serial.println(a.acceleration.z - az_offset);

  Serial.println("");
  
  Serial.print("GIROSCOPO X: ");
  Serial.print(g.gyro.x - gx_offset);
  Serial.print(" | ");
  Serial.print("GIROSCOPO Y: ");
  Serial.print(g.gyro.y - gy_offset);
  Serial.print(" | ");
  Serial.print("GIROSCOPO Z: ");
  Serial.println(g.gyro.z - gz_offset);
}

void calibracion() {
  size_t buffer_size = 2000;
  sensors_event_t a, g, temp;

  imprimir_parametros();

  Serial.println("Iniciando calibración.");
  Serial.println("");

  float ax_buf = 0, ay_buf = 0, az_buf = 0, gx_buf = 0, gy_buf = 0, gz_buf = 0;

  for(int k = 0; k < buffer_size + 100; k++){
    if(k < 100)
      continue;

    mpu.getEvent(&a, &g, &temp);
    ax_buf += a.acceleration.x;
    ay_buf += a.acceleration.y;
    az_buf += a.acceleration.z - 9.8;
    gx_buf += g.gyro.x;
    gy_buf += g.gyro.y;
    gz_buf += g.gyro.z;
  }

  ax_offset = ax_buf/buffer_size;
  ay_offset = ay_buf/buffer_size;
  az_offset = az_buf/buffer_size;

  gx_offset = gx_buf/buffer_size;
  gy_offset = gy_buf/buffer_size;
  gz_offset = gz_buf/buffer_size;

  Serial.println("Calibración finalizada. Resultados:");
  imprimir_parametros();
}