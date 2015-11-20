
#include <Wire.h>
#define MPU 0x68   //Direccion I2C de la IMU
//Constantes para conversion de unidades
#define A_R 16384.0
#define G_R 131.0
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
//Volores entrgados por el sensor
int16_t AcX, AcY, AcZ, GyX;   //aceleracion en X,Y,Z y giro en X

//Angulos
float AnguloX;
float GX;        //giro en X
float AF;        //angulo filtrado
int IN3 = 5;    //pin de entrada al puente H
int IN4 = 4;    //pin de entrada al puente H
int ENB = 3;    // enable del puente H
float U;          //ley de control
float KI=1.0143;        //K relacionada con theta
float KII=0.337;      //K relacioada con theta punto
const int stepsPerRevolution = 20;  // cambie este valor por el numero de pasos de su motor

// inicializa la libreria 'stepper' en los pines 8 a 11
void setup() {
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //Serial.begin(9600);
  

}

void loop() {
  //Leer los valores del Acelerometro de la IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //A partir del 0x3B, se piden 6 registros
  AcX = Wire.read() << 8 | Wire.read(); //Cada valor ocupa 2 registros
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  AnguloX = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG; //Se calcula el angulo
  Wire.beginTransmission(MPU);       //Leer los valores del Giroscopio
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true); //se piden dos registros
  GyX = Wire.read() << 8 | Wire.read();
  GX = -GyX / G_R;        //Calculo del angulo del Giroscopio
  AF = 0.85 * (AF + GX * 0.015) + 0.15 * AnguloX;     //Filtro
  AF=AF+1.34; 
  //Serial.print(" Angle XCF: "); Serial.print(AF);
  //Serial.print(" Giro : "); Serial.print(GX);
  //Serial.print("  U  ");Serial.print(U); Serial.print("\n");
  
  
  
  
    U=-KI*AF-KII*GX;
  if(U>=0)
 {
   U=map(U,0,10,35,50);
   digitalWrite(4,LOW);
   digitalWrite(5,HIGH);
   if(abs(AF)>=4)
   {
    U=255;
    analogWrite(ENB,U);
   }
   analogWrite(ENB,U);
 } 
   if(U<0)
 {
   U=map(abs(U),0,10,35,50);
   digitalWrite(5,LOW);
   digitalWrite(4,HIGH);
   if(abs(AF)>=4)
   {
   U=255;
    analogWrite(ENB,U);
   }
   analogWrite(ENB,U);
 }
 delay(10); //Nuestra dt sera, pues, 0.010, que es el intervalo de tiempo en cada medida
}
