#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define frec 50
#define servo 9
#define limInf_servo  500
#define limSup_servo  2500
#define ANGULO_REFERENCIA_CERO 83

#define Kp -1 // -0.8 para el PD
#define Ki -0.1
#define Kd -0.002
#define T 0.02

#define Kp_2 -0.5 //-0.7 
#define Ki_2 -3 //-3
#define Kd_2 //-0.002

int contador = 0;


Adafruit_MPU6050 mpu;
unsigned long start_time, finish_time;
float angulo_x_giros, angulo_x_aceler, angulo_estimado;

int potenciometro = A0;
int val, val_deg, val_servo;

void setup(void) {
  Serial.begin(115200);
  
  inicializarIMU();

  pinMode(servo,OUTPUT);  
  inicializarServo(0);

  inicializarAngulos();

}

void loop() {
  static float phi_ref = 0;
  static float y[2] = {0,0}; // theta es el primer subindice, phi el segundo
  static float e_theta[2] = {0,0}, u[2] = {0,0}, e_phi[2] = {0,0}, theta_ref[2] = {0,0}; // el error anterior tiene argumento 0, el actual, 1
  start_time = millis();
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  calculoAngulo(a,g);
  y[0] = angulo_estimado * 180 /PI;
  y[1] = anguloPote();

  e_phi[0] = e_phi[1];
  e_phi[1] = phi_ref - y[1];

  //Serial.print("e_phi: ");
  //Serial.println( e_phi[1]);

  theta_ref[0] = theta_ref[1];
  theta_ref[1] = controlador2PI(e_phi[1],e_phi[0],theta_ref[0]); //salida del controlador externo
  //theta_ref[1] = Kp_2 * e_phi[1];
 // Serial.print("theta_ref: ");
 // Serial.println(theta_ref[1]);
 
  e_theta[0] = e_theta[1];
  e_theta[1] = theta_ref[1] - y[0];
  u[0] = u[1];

  //u[0] = Kp*e_theta[0];
  //u[0] = controladorPIBilineal(e,e_1,u_1);
  u[1] = controladorPDBilineal(e_theta[1],e_theta[0],u[0]); 

  //Serial.print("u: ");
  //Serial.println(u[1]);

  moverServo(u[1]);
  
  Serial.println(anguloPote());

 if(contador > 1500){
   phi_ref = 0;
   contador = 0;
}

 else {

 if(contador > 1000){
  phi_ref = -10;
 }

 else{
  if(contador > 500){
    phi_ref = 30;
  }
  }
  }
  

  contador++;
 
 
  //Serial.print("y: ");
  //Serial.println(y[0]);

 elegirFrecuencia(start_time);
}


void elegirFrecuencia(unsigned long start_time){
  finish_time = millis();
  unsigned long elapsed_time = finish_time - start_time;
  if (elapsed_time < 1000 / frec) {
      delay(1000 / frec - elapsed_time);
  }
}

// La primer medicion del angulo la hacemos con los acelerometros que tienen la referencia de la gravedad
void inicializarAngulos(){
  sensors_event_t a_inicial, g_inicial, temp_inicial;
  mpu.getEvent(&a_inicial, &g_inicial, &temp_inicial);
  angulo_x_aceler = atan2(a_inicial.acceleration.y,a_inicial.acceleration.z);
  angulo_estimado = angulo_x_aceler;
  angulo_x_giros = angulo_x_aceler;
  
}

// Calculamos el ángulo en el eje x a partir del giróscopo y los acelerómetros y aplicamos un filtro complementario
// El ángulo que se obtiene con las mediciones de los giróscopos no es preciso a largo plazo por el error al integrar el sesgo  
// El obtenido con los acelerometros tiene problemas a altas frecuencias
void calculoAngulo(sensors_event_t a,sensors_event_t g){
  angulo_x_giros = angulo_estimado + g.gyro.x * 1/frec;
  angulo_x_aceler = atan2(a.acceleration.y,a.acceleration.z);
  angulo_estimado = 0.02 * angulo_x_aceler + 0.98 * angulo_x_giros;
}

void inicializarIMU(){
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("IMU OK");
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  // Muestreamos los datos de rotación y aceleración a 100Hz, por lo que podemos recontruir una señal de hasta 50Hz. 
  // Nos interesa mantener frecuencias menores a 50Hz por lo que elegimos un bw de 94Hz.
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ); 

}

//La salida del PWM es por el pin 9 
void inicializarServo(int anguloInicial){ //configuracion del timer 1
  TCCR1A = 0; 
  TCCR1A |= (1 << COM1A1);
  TCCR1B = 0;
  TCCR1B |= (1 << CS11) | (1 << WGM13);  //PWM con fase y frecuencia correctas
    
  TCNT1  = 0;
  
  OCR1A = map(anguloInicial+ANGULO_REFERENCIA_CERO, 0, 180, limInf_servo, limSup_servo); //con la frecuencia del Arduino de 16M, 900 y 2400 son los valores límites para llevar el servo de 0° a 120° (máximo rango de movimiento)
  
  ICR1 = 20000; //con la frecuencia del Arduino de 16M, 20000 es el valor necesario para que la frecuencia del PWM sea de 50Hz
}

void moverServo(int anguloDesplazamiento){
  if (anguloDesplazamiento > 45){
    anguloDesplazamiento = 45;
  } 
  if (anguloDesplazamiento < -45){
    anguloDesplazamiento = -45;
  } 
  
  val_servo = map(anguloDesplazamiento+ANGULO_REFERENCIA_CERO, 0, 180, limInf_servo, limSup_servo);
  OCR1A = val_servo;
}

float controladorPIBilineal(float e, float e_1, float u_1){
  float u = (Kp + Ki * T/2)*e + (Ki * T/2 - Kp)*e_1 + u_1;
  return u;
}


float controladorPDBilineal(float e, float e_1, float u_1){
  float u = (Kp + Kd * 2/T)*e + ( Kp - Kd * 2/T )*e_1 - u_1;
  return u;
}

float anguloPote(){
  float angulo_pote = analogRead(potenciometro);
  angulo_pote = map(angulo_pote, 461, 790, -45, 45);
  return angulo_pote ;
}

float controlador2PI(float e, float e_1, float u_1){
  float u = (Kp_2 + Ki_2 * T/2)*e + (Ki_2 * T/2 - Kp_2)*e_1 + u_1;
  return u;
}
