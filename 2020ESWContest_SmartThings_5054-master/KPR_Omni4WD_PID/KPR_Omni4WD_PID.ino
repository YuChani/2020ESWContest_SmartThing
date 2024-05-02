#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <EEPROM.h>
#define _NAMIKI_MOTOR  //for Namiki 22CL-103501PG80:1
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>

#include<Servo.h>

// #include <SoftwareSerial.h>
// SoftwareSerial mySerial(10, 11); // RX, TX

Servo sv1;
Servo sv2;

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);


Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);

boolean toggle = false;

void toggleLED()
{
  if(toggle == false)
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);
  toggle = toggle ? false : true;
}

void mecanum_wheel()
{
  if(strstr(string, "g"))
  {
    Omni.setCarAdvance(0);  
    Omni.setCarSpeedMMPS(speedMMPS, uptime);
    //Omni.delayMS(duration, debug);
    //Omni.setCarSlow2Stop(uptime);  
  }
  else if(strstr(string, "b"))
  {
    Omni.setCarBackoff(0);
    Omni.setCarSpeedMMPS(speedMMPS, uptime);
    //Omni.delayMS(duration, debug);
    //Omni.setCarSlow2Stop(uptime);  
  }
  else if(strstr(string, "l"))
  {
    Omni.setCarLeft(0);
    Omni.setCarSpeedMMPS(speedMMPS, uptime);
    //Omni.delayMS(duration, debug);
    //Omni.setCarSlow2Stop(uptime); 
  }
  else if(strstr(string, "r"))
  {
    Omni.setCarRight(0);
    Omni.setCarSpeedMMPS(speedMMPS, uptime);
    //Omni.delayMS(duration, debug);
    //Omni.setCarSlow2Stop(uptime); 
  }

  else if(strstr(string, "q"))
  {
    Omni.setCarRotateLeft();
    Omni.setCarSpeedMMPS(speedMMPS_R, uptime);
    //Omni.setCarRotateLeft(speedMMPS_R);
    //Omni.delayMS(duration, debug);
    //Omni.setCarSlow2Stop(uptime);  
  }
  else if(strstr(string, "w"))
  {
    Omni.setCarRotateRight();
    Omni.setCarSpeedMMPS(speedMMPS_R, uptime);
    //Omni.setCarRotateRight(speedMMPS_R);
   // Omni.delayMS(duration, debug);
   // Omni.setCarSlow2Stop(uptime);
   
  }
  else if(strstr(string, "t"))
  {
    toggleLED();
  }

  else if(strstr(string, "s"))
  {
    Omni.setCarSpeedMMPS(0,uptime); 
    Omni.setCarSlow2Stop(uptime);    
    Omni.setCarStop();            // 완전히 멈춘다. 
  }
}

void table_lift()
{
  if(strstr(string, "u"))
  {
    sv1.write(180);
    sv1.write(180);
    delay(2000);
    Omni.setCarBackoff(0);
    Omni.setCarSpeedMMPS(speedMMPS, uptime);
  }

  else if(strstr(string, "d"))
  {
    sv1.write(0);
    sv1.write(0);
    delay(2000);
    Omni.setCarSpeedMMPS(0,uptime); 
    Omni.setCarSlow2Stop(uptime);    
    Omni.setCarStop();
  }
}

void setup() 
{
  //TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  Omni.PIDEnable(0.35,0.04,0,50);

  sv1.attach(20);
  sv2.attach(21);
  
  Serial.begin(115200);
  pinMode(13, OUTPUT);   
}

void loop() {
  // Omni.demoActions(100,1500,500,false);
  
  char val = 0;
  char string[256];
  int i = 0;
  while(Serial.available())
  {
    char ch = Serial.read();
    string[i] = ch;
    i++;
    if(ch == -1)
      break;
    delay(5);
  }
  string[i] = 0;

  mecanum_wheel();
  table_lift();
  
  unsigned int speedMMPS = 40;
  unsigned int speedMMPS_R= 30;
  unsigned int duration = 100;
  unsigned int uptime = 400;
  bool debug = false;
 // Omni.setCarStop();            // 완전히 멈춘다. 
 // Omni.delayMS(duration);    // 지연을 둔다. 데모는 1.5초이다.  
}
