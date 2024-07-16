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

int LIMITE_ANGULO_SUPERIOR = 135;
int LIMITE_ANGULO_INFERIOR = 45;

sensors_event_t a, g, temp;
float ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;

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

float theta = 0;
float dtheta = 0;
float phi = 0;

float velocidad = 0;

float alpha = 0.98;

// ESTIMADORES [{k, k+1}]
float theta_est[2] = {0, 0};
float dtheta_est[2] = {0, 0};
float phi_est[2] = {0, 0};
float dphi_est[2] = {0, 0};

float phi_ref = 90;

float Ad[4][4] = {{1, 0.01, 0, 0}, {-0.5765, 0.9895, -2.1322, -0.2132}, {0, 0, 1, 0.01}, {0, 0, -2.5, 0.75}};
float Bd[4] = {0, 2.1323, 0, 2.50};
float Cd[2][4] = {{1, 0, 0, 0}, {0, 0, 1, 0}};
float L[4][2] = {{1.6395, -0.2132}, {65.7085, -31.7607}, {0, 1.4}, {0, 30.5}};
float F[2] = {0.5, 0}

//float Ad[4][4] = {{1.0000, 0.0100, 0.0000, 0.0000},{-0.6500, 0.9739, -2.0454, -0.2147},{0.0000, 0.0000, 1.0000, 0.0100},{0.0000, 0.0000, -2.9220, 0.6933}};
//float Bd[4] = {0, 2.0454, 0, 2.9220};
//float Cd[2][4] = {{1,0,0,0},{0,0,1,0}};
//float L[4][2] = {{1.5155, -0.2147},{54.7727, -27.9965},{0.0000, 1.2349},{0.0000, 18.5863}};

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

  OCR1A = angulo_a_servo(90);

  delay(2000);
  calibracion();
  delay(500);
}

void loop() {
  // Utilizamos millis() en lugar de micros() porque esta última llega hasta 65536 (2^16) y necesitaríamos del orden de los 10^6 para 10 Hz
  tiempo_inicial = micros();

  while (Serial.available() > 0) {
    phi_ref = Serial.read();
  }

  OCR1A = angulo_a_servo(phi_ref);

  obtener_angulo_giroscopo_x();
  obtener_angulo_accel_x();
  
  angulo_x = alpha*angulo_gir_x + (1-alpha)*angulo_accel_x;
  velocidad = w_giro_x;

  theta = angulo_x;
  dtheta = w_giro_x;
  phi = convertir_a_angulo(analogRead(PIN_POTE));
  
  theta_est[1] = Ad[0][0] * theta_est[0] + Ad[0][1] * dtheta_est[0] + Ad[0][2] * phi_est[0] + Ad[0][3] * dphi_est[0] + L[0][0] * (theta - (Cd[0][0] * theta_est[0] + Cd[0][1] * dtheta_est[0] + Cd[0][2] * phi_est[0] + Cd[0][3] * dphi_est[0])) + L[0][1] * (phi - (Cd[1][0] * theta_est[0] + Cd[1][1] * dtheta_est[0] + Cd[1][2] * phi_est[0] + Cd[1][3] * dphi_est[0])) + Bd[0] * phi_ref;

  dtheta_est[1] = Ad[1][0] * theta_est[0] + Ad[1][1] * dtheta_est[0] + Ad[1][2] * phi_est[0] + Ad[1][3] * dphi_est[0] + L[1][0] * (theta - (Cd[0][0] * theta_est[0] + Cd[0][1] * dtheta_est[0] + Cd[0][2] * phi_est[0] + Cd[0][3] * dphi_est[0])) + L[1][1] * (phi - (Cd[1][0] * theta_est[0] + Cd[1][1] * dtheta_est[0] + Cd[1][2] * phi_est[0] + Cd[1][3] * dphi_est[0])) + Bd[1] * phi_ref;

  phi_est[1] = Ad[2][0] * theta_est[0] + Ad[2][1] * dtheta_est[0] + Ad[2][2] * phi_est[0] + Ad[2][3] * dphi_est[0] + L[2][0] * (theta - (Cd[0][0] * theta_est[0] + Cd[0][1] * dtheta_est[0] + Cd[0][2] * phi_est[0] + Cd[0][3] * dphi_est[0])) + L[2][1] * (phi - (Cd[1][0] * theta_est[0] + Cd[1][1] * dtheta_est[0] + Cd[1][2] * phi_est[0] + Cd[1][3] * dphi_est[0])) + Bd[2] * phi_ref;

  dphi_est[1] = Ad[3][0] * theta_est[0] + Ad[3][1] * dtheta_est[0] + Ad[3][2] * phi_est[0] + Ad[3][3] * dphi_est[0] + + L[3][0] * (theta - (Cd[0][0] * theta_est[0] + Cd[0][1] * dtheta_est[0] + Cd[0][2] * phi_est[0] + Cd[0][3] * dphi_est[0])) + L[3][1] * (phi - (Cd[1][0] * theta_est[0] + Cd[1][1] * dtheta_est[0] + Cd[1][2] * phi_est[0] + Cd[1][3] * dphi_est[0])) + Bd[3] * phi_ref;

  theta_est[0] = theta_est[1];
  dtheta_est[0] = dtheta_est[1];
  phi_est[0] = phi_est[1];
  dphi_est[0] = dphi_est[1];
  
  matlab_send(theta, theta_est[0], velocidad, dtheta_est[0], phi, phi_est[0], dphi_est[0]);

  tiempo_final = micros();
  delayMicroseconds(periodo_lectura - (tiempo_final - tiempo_inicial));

  
}


void config_50_hz(){ 
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = TOP;
}



void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
  b = (byte *) &dato4;
  Serial.write(b,4);
  b = (byte *) &dato5;
  Serial.write(b,4);
  b = (byte *) &dato6;
  Serial.write(b,4);
  b = (byte *) &dato7;
  Serial.write(b,4);
}

void obtener_angulo_giroscopo_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  w_giro_x = (g.gyro.x-offset_giro_x) * 180 / M_PI;  
  angulo_gir_x = angulo_x + (g.gyro.x-offset_giro_x) * (periodo_lectura/1e6) * 180 / M_PI; 
}

void obtener_angulo_accel_x() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  angulo_accel_x  = atan2(a.acceleration.y-offset_accel_y, a.acceleration.z-offset_accel_z) * 180 / M_PI;
}

int convertir_a_angulo(int valor_pote){
  return map(valor_pote, MINIMO_POTE, MAXIMO_POTE, 0, 180);
}

void calibracion() {
  size_t buffer_size = 2000;
  imprimir_parametros();

  sensors_event_t a, g, temp;

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

  Serial.println("Calibración finalizada.");
  Serial.println("");
  imprimir_parametros();
  Serial.println("");
}

int angulo_a_servo(float angulo){
  return map(angulo, 0, 180, MINIMO, MAXIMO);
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