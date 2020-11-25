//  6 Channel Receiver | 6 Kanal Alıcı
//  PWM output on pins D2, D3, D4, D5, D6, D7 (Çıkış pinleri)
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;
Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;
struct Signal {
byte throttle;      
byte pitch;
byte roll;
byte yaw;
byte aux1;
byte aux2;
};
Signal data;
const uint64_t pipeIn = 0xE9E8F0F0E1LL;
RF24 radio(9, 10); 
void ResetData()
{
  Serial.print("reset");
  Serial.print("\n");
  
// Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
// The middle position for Potenciometers. (254/2=127) | Potansiyometreler için orta konum
data.roll = 127;   // Center | Merkez
data.pitch = 127;  // Center | Merkez
data.throttle = 1; // Motor Stop | Motor Kapalı
data.yaw = 127;   // Center | Merkez
data.aux1 = 127;   // Center | Merkez
data.aux2 = 127;   // Center | Merkez
}


//drone

Servo esc_Front_Left;
Servo esc_Front_Right;
Servo esc_Rear_Left;
Servo esc_Rear_Right;


int pin_Front_Left = 2; // this is the output of the arduino
int pin_Front_Right = 3; // this is the output of the arduino
int pin_Rear_Left = 4; // this is the output of the arduino
int pin_Rear_Right = 5; // this is the output of the arduino

int led = 7;


int potValue; // the potentiometer value

int maxJoystickInfluence = 20; // the maximum influnce the joystick has over the motors . // must be lower than 180 // the higher this is , the more the drone will be able to tilt
//edn drone

void setup()
{
  Serial.begin(9600);
  //Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.
//  ch1.attach(2);
//  ch2.attach(3);
//  ch3.attach(4);
//  ch4.attach(5);
//  ch5.attach(6);
//  ch6.attach(7);
  //Configure the NRF24 module
  ResetData();
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening(); //start the radio comunication for receiver | Alıcı olarak sinyal iletişimi başlatılıyor
  pinMode(6,OUTPUT);
  pinMode(led,OUTPUT);


  //drone 

  
  esc_Front_Left.attach(pin_Front_Left); // (pin, min pulse width, max pulse width in microseconds)
  esc_Front_Right.attach(pin_Front_Right); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Left.attach(pin_Rear_Left); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Right.attach(pin_Rear_Right); // (pin, min pulse width, max pulse width in microseconds)
//  
}
unsigned long lastRecvTime = 0;
void recvData()
{
  while ( radio.available() ) 
    //Serial.print("available");
  radio.read(&data, sizeof(Signal));
  lastRecvTime = millis();   // receive the data | data alınıyor
  //led 
  digitalWrite(led,HIGH);
  }
}

void loop()
{
  recvData();
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
  ResetData(); // Signal lost.. Reset data | Sinyal kayıpsa data resetleniyor
  }
  ch_width_4 = map(data.yaw,      15, 255, 990, 2000);     // pin D5 (PWM signal)
  ch_width_2 = map(data.pitch,    15, 255, 990, 2000);     // pin D3 (PWM signal)
  ch_width_3 = map(data.throttle, 15, 255, 990, 2000);     // pin D4 (PWM signal)
  ch_width_1 = map(data.roll,     15, 255, 990, 2000);     // pin D2 (PWM signal)
  ch_width_5 = map(data.aux1,     15, 255, 990, 2000);     // pin D6 (PWM signal)
  ch_width_6 = map(data.aux2,     15, 255, 990, 2000);     // pin D7 (PWM signal)
  // Write the PWM signal | PWM sinyaller çıkışlara gönderiliyor
//  ch1.writeMicroseconds(ch_width_1);
//  ch2.writeMicroseconds(ch_width_2);
//  ch3.writeMicroseconds(ch_width_3);
//  ch4.writeMicroseconds(ch_width_4);
//  ch5.writeMicroseconds(ch_width_5);
//  ch6.writeMicroseconds(ch_width_6);

  //drone 
   potValue = map(data.throttle, 0 ,255, 1000, 2000);

  int speed_Front_Left = potValue;
  int speed_Front_Right = potValue;
  int speed_Rear_Left = potValue;
  int speed_Rear_Right = potValue;  
  
  
  int xValue = map(data.roll, 0, 255, -maxJoystickInfluence, maxJoystickInfluence);   // scale it to use it with the servo library (value between 0 and 180)
  int yValue = map(data.pitch, 0, 255, maxJoystickInfluence, -maxJoystickInfluence);   // scale it to use it with the servo library (value between 0 and 180)

  if(potValue>1100){
  if(xValue < 0)
  {
    speed_Front_Left += xValue;
    speed_Rear_Left += xValue;
  }
  else
  {
    speed_Front_Right -= xValue;
    speed_Rear_Right -= xValue;
  }
  
  if(yValue > 0)
  {
    speed_Front_Left -= yValue;
    speed_Front_Right -= yValue;
  }
  else
  {
    speed_Rear_Left += yValue;
    speed_Rear_Right += yValue;
  }
  }

  esc_Front_Left.writeMicroseconds(speed_Front_Left);    // Send the signal to the ESC
  esc_Front_Right.writeMicroseconds(speed_Front_Right);    // Send the signal to the ESC
  esc_Rear_Left.writeMicroseconds(speed_Rear_Left);    // Send the signal to the ESC
  esc_Rear_Right.writeMicroseconds(speed_Rear_Right);    // Send the signal to the ESC

  //end drone


  
  
  Serial.print("speed_Front_Left: ");
  Serial.print(speed_Front_Left);
  Serial.print("\t");
  Serial.print("speed_Front_Right: ");
  Serial.print(speed_Front_Right);
  Serial.print("\t");
  Serial.print("speed_Rear_Left: ");
  Serial.print(speed_Rear_Left);
  Serial.print("\t");
  Serial.print("speed_Rear_Right: ");
  Serial.print(speed_Rear_Right);
  Serial.print("\n");



//  
}


 
