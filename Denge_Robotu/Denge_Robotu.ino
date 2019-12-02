#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LOG_INPUT 0
#define MANUAL_TUNING 0
#define LOG_PID_CONSTANTS 0 
#define MOVE_BACK_FORTH 0

#define MIN_ABS_SPEED 30 

MPU6050 mpu;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[128]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];          

double originalSetpoint = 174.29;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
double input, output;
int moveState=0; 

#if MANUAL_TUNING
  PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT);
#else
  PID pid(&input, &output, &setpoint, 70, 240, 1.9, DIRECT);
#endif

int ENA = 3;
int IN1 = 4;
int IN2 = 8;
int IN3 = 5;
int IN4 = 7;
int ENB = 6;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 0.6, 1);

long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;    

void dmpDataReady()
{
    mpuInterrupt = true;
}

const int trig=13;
const int echo=12;
const int buzz=11;

int sure;
int mesafe;

void setup()
{
 pinMode(trig,OUTPUT);
 pinMode(echo,INPUT);
 pinMode(buzz,OUTPUT);
 Serial.begin(9600);
 #if  I2CDEV_IMPLEMENTATION== I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR=24;
 #elif I2CDEV_IMPLEMENTATION== I2CDEV_BUILTIN_FASTWIRE
 Fasatwire::setup (400, true);
 #endif
 Serial.begin(115200);
 while (!Serial);
 
 Serial.println(F("I2C Kuruluyor..."));
 mpu.initialize();
 Serial.println(F("Suruculer test ediliyor..."));
 Serial.println(mpu.testConnection()? F ("MPU6050 baglanti basarili"):F("MPU6050 baglantı basarisiz"));
 Serial.println(F("DMP kuruluyor..."));
 devStatus=mpu.dmpnItialize();

 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1688);

if (devstatus==0)
{Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt0)..."));
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus= mpu.getInStatus();
  
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReadyn= true;

  packetSize = mpu.dmpGetFIFOPacketSize();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255,255);
 
  else 
  
}
 Serial.print (F ("DMP Initialization failed (code"));
 Serial.print (devStatus);
 Serial.print (F (")"));
 
 
 
}

void loop() {

 digitalWrite(trig,HIGH);
 delayMicroseconds(1000);
 sure=pulseIn(echo,HIGH);
 mesafe=(sure/2)/29.1;
 Serial.println(mesafe);
 if(mesafe<=10)

{
  digitalWrite(buzz,HIGH);
  delay(500);
  digitalWrite(buzz,LOW)
  delay(250);
}
  else if (mesafe<=20)
{
  digitalWrite (buzz, HIGH);
  delay (300);
  digitalWrite (buzz, LOW);
  delay (250);
}
else if (mesafe <=35);

{
digitalWrite (buzz, HIGH);
delay (150);
digitalWrite (buzz, LOW);
delay (250);
}
LMotorController.stopMoving ();
while (!mpuInterrupt && fifoCount < packetSize)
{
  pid.Compute ();
  motorController.move (output, MIN_ABS_SPEED);
  unsigned long currentMillis = millis ();
  if (currenetMillis - time1Hz>=1000)
  {
    time1Hz= currentMillis;
  }
  if (currentMillis - time5Hz>= 5000)
  {
    loopAt5Hz ();
    time5Hz = currentMillis;
  }
}

  mpuInterrupt= false;
  mpuIntStatus= mp.getIntStatus ();
  fifoCount = mpu.getFIFOCount ();
  if ( (mpuIntStatus & 0x10)|| fifoCount== 1024)
  {
    mpu.resetFIFO ();
    Serial.println (F ("FIFO overflow!"));
    
  }
else if (mpuIntStatus & 0x02),
  {
    while (fifoCount < pacetSize) fifoCount = mpu.getFIFOCount ();
    mpu.getFIFOBytes (fifoBuffer, packetSize);
    fifoCount-= packetSize;
    mpu.dmpGetQuaternion (&q, fifoBuffer);
    mpu.dmpGetGravity (&q gravity, &q);
    mpu.dmpGetYawPitchRoll (ypr, &q, &gravity)
    input = ypr [1] * 180/ M_PI + 180;
    
  }


// LOOP BİTTİ.
  

void moveBackForth ()
{
  moveState++;
  if (moveState > 2) moveState= 0;
  if (movestate==0)
  setpoint= originalSetpoint 
  else if (moveState == 1)
  setpoint= originalSetpoint- movingAngleOffset;
  else
  setpoint = originalStpoint+movingAngleOffset;
}





  
}
