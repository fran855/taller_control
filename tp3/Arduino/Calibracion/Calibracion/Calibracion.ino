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
#define FRECUENCIA_LECTURA 50

uint16_t periodo_lectura;
uint16_t tiempo_inicial = 0;
uint16_t tiempo_final = 0;
sensors_event_t a, g, temp;
float ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;

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
  OCR1A = 5100;
  delay(3000);
  
  calibracion();
  
}

void loop() {
  // Utilizamos millis() en lugar de micros() porque esta última llega hasta 65536 (2^16) y necesitaríamos del orden de los 10^6 para 10 Hz
  tiempo_inicial = micros();

  tiempo_final = micros();
  delayMicroseconds(periodo_lectura - (tiempo_final - tiempo_inicial));
}


void config_50_hz(){ 
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = TOP;
}

void calibracion() {
  size_t buffer_size = 2000;

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

void imprimir_parametros(){
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

