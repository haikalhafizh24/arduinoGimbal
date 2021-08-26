#include <Wire.h>

//Deklarasi Variable//

#define SDA 5
#define SCL 4 

int fmotor = 18;
int rmotor = 19;
int mspeed = 0;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX;

float Acceleration_angle;
float Gyro_angle;
float Total_angle;

float elapsedTime, waktu, waktuPrev;
float rad_to_deg = 180/3.141592654;

float PID, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

//Nilai Konstan PID//
float kp=2.4;
float ki=2;
float kd=0.2;

//Set Point //
float desired_angle = 0;
void setup() 
{
  
//Untuk Memulai Komunikasi I2C//
  Wire.begin(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  pinMode(fmotor,OUTPUT);
  pinMode(rmotor,OUTPUT);
  
  Serial.begin(9600);

//Menghitung Waktu (Milisecond)//
  waktu = millis(); 
}

void loop() 
{
    waktuPrev = waktu;  
    waktu = millis();  
    elapsedTime = (waktu - waktuPrev) / 1000; //dibagi 1000 jadi Sec//
    
    Wire.beginTransmission(0x68); //Alamat Register Sensor MPU//
    Wire.write(0x3B); //Mulai dari register pertama 0x3B (ACCEL_XOUT)//
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);  //Baca 3 register, setiap sumbu disimpan di 2 register//
    
//Membaca nilai Accelerometer//
    Acc_rawX=Wire.read()<<8|Wire.read(); 
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read(); 
    
//Konversi nilai Accelerometer ke nilai Sudut//
    Acceleration_angle= atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    //Untuk rentang  +-2g, nilai RAW dibagi 16384.0, Sesuai datasheet

    Wire.beginTransmission(0x68);
    Wire.write(0x43); //Mulai dari register pertama 0x43(GYRO_XOUT)//
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,2,true); //Baca 1 register, setiap sumbu disimpan di 2 register//
    
//Membaca nilai Giroskop//
    Gyr_rawX=Wire.read()<<8|Wire.read();
    
//Konversi nilai Giroskop ke nilai Sudut//
    Gyro_angle = Gyr_rawX/131.0; 
    //Untuk rentang 250deg/s, nilai RAW dibagi 131,0, Sesuai datasheet
    
//Menggabungkan kedua Sudut menggunakan COMPLEMENTARY FILTER//
    Total_angle = 0.98 *(Total_angle + Gyro_angle*elapsedTime) + 0.02*Acceleration_angle;
    
//TOTAL_ANGLE Sumbu Pitch yang Digunakan//

//ERROR//
    error = Total_angle - desired_angle; 
    
//PROPORTIONAL ERROR//
    pid_p = kp*error;
    
//INTERGRAL ERROR//
    pid_i = pid_i+(ki*error);  
    
//DIFFERENTIAL ERROR//
    pid_d = kd*((error - previous_error)/elapsedTime);
    
//Total Nilai PID//
    PID = pid_p + pid_d + pid_i;
    
//Update Nilai ERROR//
    previous_error = error;
    
    //Serial.println(PID);
    //delay(60);                   
    //Serial.println(Total_angle); 
    
//Konversi Nilai PID menjadi PWM//
    mspeed = abs(PID);
    //Serial.println(mspeed);
    
//Arah Putaran//
    if(Total_angle<0)
      {
       clockw(); 
      }
    if(Total_angle>0)
      {
       anticlockw();
      }
    if(PID>45){
      stop();
    }
    if(PID<-45){
      stop();  
    } 
}

//Gerakan Motor//
void clockw()
{
  analogWrite(fmotor,mspeed);
  analogWrite(rmotor,0);
}
void anticlockw()
{
  analogWrite(fmotor,0);
  analogWrite(rmotor,mspeed);
}
void stop()
{
  analogWrite(fmotor,0);
  analogWrite(rmotor,0);
}
