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
#define MAXIMO_POTE 682
#define MINIMO 1100
#define MAXIMO 5000
#define TOP 39999
#define FRECUENCIA_LECTURA 100

uint16_t tiempo_inicial = 0;
uint16_t tiempo_final = 0;

float offset_giro_x = -0.04;
float offset_giro_y = 0.03;
float offset_giro_z = 0;

float offset_accel_x = 1.13;
float offset_accel_y = 0.39;
float offset_accel_z = -0.75;

float angulo_gir_x = 0;
float angulo_accel_x = 0;
float angulo_x = 0;

float alpha = 0.98;

const long int periodo_lectura = 10000;

void setup(void) {
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

  // Config Servo y PWM
  pinMode(PIN_PWM, OUTPUT);
  config_50_hz();
	delay(100);
}

void loop() {
	float d1=0, d2=0, d3=0;

  tiempo_inicial = micros();
  int valor_pote = analogRead(PIN_POTE);
  
  obtener_angulo_giroscopo_x();
  obtener_angulo_accel_x();
  
  angulo_x = alpha*angulo_gir_x + (1-alpha)*angulo_accel_x;

  d1 = angulo_x;
  d2 = valor_pote;
  OCR1A = 3050;
  //OCR1A = procesar_valor_pote(valor_pote);

  tiempo_final = micros();
  delayMicroseconds(periodo_lectura - (tiempo_final - tiempo_inicial));

	// Matlab send
  matlab_send(d1,d2,d3);
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

void obtener_angulo_giroscopo_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  angulo_gir_x = angulo_x + (g.gyro.x-offset_giro_x) * (periodo_lectura/1e6) * 180 / M_PI; 
}

void obtener_angulo_accel_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_accel_x  = atan2(a.acceleration.y-offset_accel_y, a.acceleration.z-offset_accel_z) * 180 / M_PI;
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
