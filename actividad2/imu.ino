#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

uint16_t tiempo_inicial = 0;
uint16_t tiempo_final = 0;
uint16_t prev_time = 0;
uint16_t current_time = 0;

float offset_giro_x = -0.04;
float offset_giro_y = 0.03;
float offset_giro_z = 0;

float offset_accel_x = 1.13;
float offset_accel_y = 0.39;
float offset_accel_z = -0.75;

float dt = 0;

float angulo_gir_x = 0;
float angulo_accel_x = 0;

const long int periodo_lectura = 10000;

void setup(void) {
	Serial.begin(115200);

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

	delay(100);
}

// Qué pasa si un timer desborda y el otro no?
void loop() {
  tiempo_inicial = micros();
  
  current_time = micros();
  dt = (current_time - prev_time) / 1e6;
  obtener_angulo_accel_x();
  prev_time = current_time;
  

  tiempo_final = micros();
  delayMicroseconds(periodo_lectura - (tiempo_final - tiempo_inicial));
}

void obtener_angulo_giroscopo_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  angulo_gir_x = angulo_gir_x + (g.gyro.x-offset_giro_x) * dt * 180 / M_PI; 

  Serial.print("Ángulo x: ");
  Serial.println(angulo_gir_x);
}

void obtener_angulo_accel_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Accel y: ");
  Serial.println(a.acceleration.y);
  Serial.print("Accel z: ");
  Serial.println(a.acceleration.z);

  angulo_accel_x  = atan2(a.acceleration.y-offset_accel_y, a.acceleration.z-offset_accel_z) * 180 / M_PI;

  Serial.print("Ángulo x: ");
  Serial.println(angulo_accel_x);
}
