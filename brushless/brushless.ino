 

#include <SoftwareSerial.h>
#include <Servo.h>

Servo esc5;

SoftwareSerial bt(0,1);

void setup() {
  // put your setup code here, to run once:

  esc5.attach(5,1000,2000);
  esc5.writeMicroseconds(1000);
  delay(1000);

  bt.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val;
  if(bt.available())
  {
    val = bt.read();
    Serial.print(val);
    Serial.print("  ");
    val = map(val,0,180,1000,2000);
    Serial.print(val);
    Serial.print(" \n");
    
    
    esc5.writeMicroseconds(val);
    delay(2000);
  }
}
