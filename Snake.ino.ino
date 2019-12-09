#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
//Some of the variables here may not used as they were ment for functions that have not been implimented yet
int counter=0;
int counter1=0;
int counter2=0;
const byte row[]={3,4,5,6,7};
const byte col[]={8,9,10,11,12};
byte state=0;
byte top=4;
byte applex;
byte appley;
byte gen=0;
int x_val[]={2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int y_val[]={3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int i,j,k;
float a,b,c;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() 
{
    mpuInterrupt = true;
}
int generate_apple()
{
    marker:
    applex= (rand()%5);
    appley= (rand()%5);
    Serial.print(applex);
    Serial.print("  ");
    Serial.println(appley);
    for(k=0;k<top;k++)
    {
      if(applex==x_val[k] && appley==y_val[k])
      {
        goto marker;
      }
    }
}
int light(int x,int y)
{
  digitalWrite(col[x],HIGH);
  digitalWrite(row[y],LOW);
}
int nlight(int x,int y)
{
  digitalWrite(col[x],LOW);
  digitalWrite(row[y],HIGH);
}
int show()
{
  if(counter2>=15)
  {
    light(applex,appley);
    nlight(applex,appley);
    counter2=0;
  }
  
  if(counter1>=4)
  {
    light(x_val[0],y_val[0]);
    nlight(x_val[0],y_val[0]);
    counter1=0;
  }
  for(i=1;i<top;i++)
  {
    light(x_val[i],y_val[i]);
    nlight(x_val[i],y_val[i]);
  }
}
int memory()
{
  for(i=top-1;i>0;i--)
  {
    x_val[i]=x_val[i-1];
    y_val[i]=y_val[i-1];
  }
}
void setup() 
{
  
  pinMode(12,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  for(i=0;i<5;i++)
  {
    digitalWrite(col[i],LOW);
    digitalWrite(row[i],HIGH);
  }
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(-25);
    mpu.setYGyroOffset(11);
    mpu.setZGyroOffset(-7);
    mpu.setXAccelOffset(-3495);
    mpu.setYAccelOffset(-654);
    mpu.setZAccelOffset(1048);
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    generate_apple();
}

void loop() 
{
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) 
  {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      a=ypr[0] * 180/M_PI;
      b=ypr[1] * 180/M_PI;
      c=ypr[2] * 180/M_PI;
      /*Serial.print("ypr\t");
      Serial.print(a);
      Serial.print("\t");
      Serial.print(b);
      Serial.print("\t");
      Serial.println(c);*/
  }
  //Code till now has been copied and motified(in order to remove unnecessary data) from mpu examples
  //Following code is probably not the most optimised but I could not think of anything to make it better
  counter ++;
  counter1 ++;
  counter2 ++;
  if(counter>15)
  {
    switch(state)
    {
      case 0:
      {
        if(c>20 && y_val[0]<4)
        {
          memory();
          y_val[0]++;
        }
        if(b>20 && x_val[0]<4)
        {
          state=1;
          memory();
          x_val[0]++;
        }
        if(b<-20 && x_val[0]>0)
        {
          state=3;
          memory();
          x_val[0]--;
        }
        break;
      }
      case 1:
      {
        if(b>20 && x_val[0]<4)
        {
          memory();
          x_val[0]++;
        }
        if(c<-20 && y_val[0]>0)
        {
          state=2;
          memory();
          y_val[0]--;
        }
        if(c>20 && y_val[0]<4)
        {
          state=0;
          memory();
          y_val[0]++;
        }
        break;
      }
      case 2:
      {
        if(c<-20 && y_val[0]>0)
        {
          memory();
          y_val[0]--;
        }
        if(b>20 && x_val[0]<4)
        {
          state=1;
          memory();
          x_val[0]++;
        }
        if(b<-20 && x_val[0]>0)
        {
          state=3;
          memory();
          x_val[0]--;
        }
        break;
      }
      case 3:
      {
        if(b<-20 && x_val[0]>0)
        {
          memory();
          x_val[0]--;
        }
        if(c<-20 && y_val[0]>0)
        {
          state=2;
          memory();
          y_val[0]--;
        }
        if(c>20 && y_val[0]<4)
        {
          state=0;
          memory();
          y_val[0]++;
        }
        break;
      }
    }
    counter=0;
  }
  show();
  if(applex==x_val[0] && appley==y_val[0])
  {
    top++;
    memory();
    generate_apple();
  }
}
