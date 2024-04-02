#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
uint16_t tiempo_inicial = 0;
uint16_t tiempo_final = 0;
long int posicion_actual = 0;

const long int periodo_lectura = 100;

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

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 5-10-21-44-94-184-260 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

	delay(100);
}

void loop() {
	
  tiempo_inicial = millis();
  // float d1=0, d2=0, d3=0;
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  
  obtener_angulo_giroscopo();
  Serial.print("Est: ");
  Serial.println(posicion_actual);
  // Serial.println(g);
  tiempo_final = millis();

  delay(periodo_lectura - (tiempo_final - tiempo_inicial));
  

}

void obtener_angulo_giroscopo(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  posicion_actual = posicion_actual + g.gyro.x * periodo_lectura; 
  Serial.println(g.gyro.x);
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
