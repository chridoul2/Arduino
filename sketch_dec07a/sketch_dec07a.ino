#include <Servo.h>


Servo esc_Front_Left;
Servo esc_Front_Right;
Servo esc_Rear_Left;
Servo esc_Rear_Right;


int pin_Front_Left = 2; // this is the output of the arduino
int pin_Front_Right = 3; // this is the output of the arduino
int pin_Rear_Left = 4; // this is the output of the arduino
int pin_Rear_Right = 5; // this is the output of the arduino

int potValue; // the potentiometer value


void setup() {
  // put your setup code here, to run once:
   esc_Front_Left.attach(pin_Front_Left); // (pin, min pulse width, max pulse width in microseconds)
  esc_Front_Right.attach(pin_Front_Right); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Left.attach(pin_Rear_Left); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Right.attach(pin_Rear_Right); // (pin, min pulse width, max pulse width in microseconds)


  esc_Front_Left.writeMicroseconds(2000);    // Send the signal to the ESC
  esc_Front_Right.writeMicroseconds(2000);    // Send the signal to the ESC
  esc_Rear_Left.writeMicroseconds(2000);    // Send the signal to the ESC
  esc_Rear_Right.writeMicroseconds(2000);    // Send the signal to the ESC

  delay(10000);
   esc_Front_Left.writeMicroseconds(1000);    // Send the signal to the ESC
  esc_Front_Right.writeMicroseconds(1000);    // Send the signal to the ESC
  esc_Rear_Left.writeMicroseconds(1000);    // Send the signal to the ESC
  esc_Rear_Right.writeMicroseconds(1000);    // Send the signal to the ESC


  delay(10000);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
}
