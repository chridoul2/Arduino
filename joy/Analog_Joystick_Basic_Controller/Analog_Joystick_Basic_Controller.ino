#include <Servo.h>
#include <Wire.h>

//  6 Channel Receiver | 6 Kanal Alıcı
//  PWM output on pins D2, D3, D4, D5, D6, D7 (Çıkış pinleri)
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define potentInput A0//the input of the potentiometer

#define joyX A1
#define joyY A2

Servo esc_Front_Left;
Servo esc_Front_Right;
Servo esc_Rear_Left;
Servo esc_Rear_Right;

//receiver
Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;


//receiver channels
int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;


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
// Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
// The middle position for Potenciometers. (254/2=127) | Potansiyometreler için orta konum
data.roll = 127;   // Center | Merkez
data.pitch = 127;  // Center | Merkez
data.throttle = 12; // Motor Stop | Motor Kapalı
data.yaw = 127;   // Center | Merkez
data.aux1 = 127;   // Center | Merkez
data.aux2 = 127;   // Center | Merkez
}
//end receiver

int pin_Front_Left = 5; // this is the output of the arduino
int pin_Front_Right = 9; // this is the output of the arduino
int pin_Rear_Left = 10; // this is the output of the arduino
int pin_Rear_Right = 11; // this is the output of the arduino

int potValue; // the potentiometer value

int maxJoystickInfluence = 20; // the maximum influnce the joystick has over the motors . // must be lower than 180 // the higher this is , the more the drone will be able to tilt


//Gyro values 
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


void setup() {

  Serial.begin(9600);
  //receiver
  //Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.
  ch1.attach(2);
  ch2.attach(3);
  ch3.attach(4);
  ch4.attach(5);
  ch5.attach(6);
  ch6.attach(7);
  //Configure the NRF24 module
  ResetData();
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening(); //start the radio comunication for receiver | Alıcı olarak sinyal iletişimi başlatılıyor
  pinMode(6,OUTPUT);
  //end receiver


  
  Serial.begin(19200);//Sets the data rate in bits per second (baud) for serial data transmission
  esc_Front_Left.attach(pin_Front_Left,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  esc_Front_Right.attach(pin_Front_Right,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Left.attach(pin_Rear_Left,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Right.attach(pin_Rear_Right,1000,2000); // (pin, min pulse width, max pulse width in microseconds)

  //Gyro
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
 // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);


}

  //RECEIVER
  unsigned long lastRecvTime = 0;
  void recvData()
  {
    while ( radio.available() ) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();   // receive the data | data alınıyor
    }
  }
  //end receiver

 
void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(1);
  

  //receiver
   recvData();
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
  ResetData(); // Signal lost.. Reset data | Sinyal kayıpsa data resetleniyor
  }
  ch_width_4 = map(data.yaw,      0, 255, 1000, 2000);     // pin D5 (PWM signal)
  ch_width_2 = map(data.pitch,    0, 255, 1000, 2000);     // pin D3 (PWM signal)
  ch_width_3 = map(data.throttle, 0, 255, 1000, 2000);     // pin D4 (PWM signal)
  ch_width_1 = map(data.roll,     0, 255, 1000, 2000);     // pin D2 (PWM signal)
  ch_width_5 = map(data.aux1,     0, 255, 1000, 2000);     // pin D6 (PWM signal)
  ch_width_6 = map(data.aux2,     0, 255, 1000, 2000);     // pin D7 (PWM signal)
  ch4.writeMicroseconds(ch_width_4);
  //end receiver
  
  
//  potValue = analogRead(potentInput);   // reads the value of the potentiometer (value between 0 and 1023)
//  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)

  potValue = map(data.throttle, 0 ,255, 0, 180);

  int speed_Front_Left = potValue;
  int speed_Front_Right = potValue;
  int speed_Rear_Left = potValue;
  int speed_Rear_Right = potValue;  
  
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);
  xValue = map(xValue, 0, 1023, -maxJoystickInfluence, maxJoystickInfluence);   // scale it to use it with the servo library (value between 0 and 180)
  yValue = map(yValue, 0, 1023, maxJoystickInfluence, -maxJoystickInfluence);   // scale it to use it with the servo library (value between 0 and 180)

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


  esc_Front_Left.write(speed_Front_Left);    // Send the signal to the ESC
  esc_Front_Right.write(speed_Front_Right);    // Send the signal to the ESC
  esc_Rear_Left.write(speed_Rear_Left);    // Send the signal to the ESC
  esc_Rear_Right.write(speed_Rear_Right);    // Send the signal to the ESC

 
  //print the values with to plot or view
//  Serial.print(xValue);
//  Serial.print("\t");
//  Serial.println(yValue);

  //Gyro
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor
  Serial.print("ch4: ");
  Serial.print(data.throttle);
  Serial.print("\n");

}//loop End


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
//  Serial.print("AccErrorX: ");
//  Serial.println(AccErrorX);
//  Serial.print("AccErrorY: ");
//  Serial.println(AccErrorY);
//  Serial.print("GyroErrorX: ");
//  Serial.println(GyroErrorX);
//  Serial.print("GyroErrorY: ");
//  Serial.println(GyroErrorY);
//  Serial.print("GyroErrorZ: ");
//  Serial.println(GyroErrorZ);



}
