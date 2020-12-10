//  6 Channel Receiver | 6 Kanal Alıcı
//  PWM output on pins D2, D3, D4, D5, D6, D7 (Çıkış pinleri)
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
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

//Gyro values
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;


float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];


float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180 / 3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error, error2, previous_error2, PID_2;
float PID_new = 0;
float PID_2_new = 0;

int count  = 0;
int count_bool  = 0;
float PID_table[100] = {};

float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float pid_p_2 = 0;
float pid_i_2 = 0;
float pid_d_2 = 0;
/////////////////PID CONSTANTS/////////////////
double kp = 3.0; //3.55
double ki = 0.005; //0.003
double kd = 0.05; //2.05

double kp_2 = 3.0; //3.55
double ki_2 = 0.002; //0.003
double kd_2 = 0.0001; //2.05
///////////////////////////////////////////////

//double throttle = 1300; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
//balance to stay steady
//end gyro

//




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
  radio.openReadingPipe(1, pipeIn);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening(); //start the radio comunication for receiver | Alıcı olarak sinyal iletişimi başlatılıyor
  pinMode(6, OUTPUT);
  pinMode(led, OUTPUT);


  //drone


  esc_Front_Left.attach(pin_Front_Left); // (pin, min pulse width, max pulse width in microseconds)
  esc_Front_Right.attach(pin_Front_Right); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Left.attach(pin_Rear_Left); // (pin, min pulse width, max pulse width in microseconds)
  esc_Rear_Right.attach(pin_Rear_Right); // (pin, min pulse width, max pulse width in microseconds)
  //  end drone


  //Gyro
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // right_prop.attach(3); //attatch the right motor to pin 3
  //left_prop.attach(5);  //attatch the left motor to pin 5

  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
     of PWM to them before connecting the battery. Otherwise
     the ESCs won't start up or enter in the configure mode.
     The min value is 1000us and max is 2000us, REMEMBER!*/
  //left_prop.writeMicroseconds(1000);
  // right_prop.writeMicroseconds(1000);
  delay(7000); /*Give some delay, 7s, to have time to connect
                 the propellers and let everything start up*/

  //edn gyro



}



unsigned long lastRecvTime = 0;
void recvData()
{
  while ( radio.available() ) {
    //Serial.print("available");
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();   // receive the data | data alınıyor
    //led
    if (radio.available()) {
      digitalWrite(led, HIGH);
    }
    else{
      esc_Front_Left.writeMicroseconds(1000);    // Send the signal to the ESC
  esc_Front_Right.writeMicroseconds(1000);    // Send the signal to the ESC
  esc_Rear_Left.writeMicroseconds(1000);    // Send the signal to the ESC
  esc_Rear_Right.writeMicroseconds(1000);    // Send the signal to the ESC
  }
    
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
  potValue = map(data.throttle, 0 , 255, 1000, 1900);

  int speed_Front_Left = potValue;
  int speed_Front_Right = potValue;
  int speed_Rear_Left = potValue;
  int speed_Rear_Right = potValue;


  int xValue = map(data.roll, 0, 255, -maxJoystickInfluence, maxJoystickInfluence);   // scale it to use it with the servo library (value between 0 and 180)
  int yValue = map(data.pitch, 0, 255, maxJoystickInfluence, -maxJoystickInfluence);   // scale it to use it with the servo library (value between 0 and 180)

  if (potValue > 1100) {
    if (xValue < 0)
    {
      speed_Front_Left += xValue;
      speed_Rear_Left += xValue;
    }
    else
    {
      speed_Front_Right -= xValue;
      speed_Rear_Right -= xValue;
    }

    if (yValue > 0)
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

 

  //end drone


  //  Serial.print("data.throttle: ");
  //  Serial.print(data.throttle);
  //  Serial.print("\t");
  //  Serial.print("speed_Front_Right: ");
  //  Serial.print(speed_Front_Right);
  //  Serial.print("\t");
  //  Serial.print("speed_Rear_Left: ");
  //  Serial.print(speed_Rear_Left);
  //  Serial.print("\t");
  //  Serial.print("speed_Rear_Right: ");
  //  Serial.print(speed_Rear_Right);
  //  Serial.print("\n");



  //Gyro
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;

  /*The tiemStep is the time that elapsed since the previous loop.
     This is the value that we will use in the formulas as "elapsedTime"
     in seconds. We work in ms so we haveto divide the value by 1000
    to obtain seconds*/

  /*Reed the values that the accelerometre gives.
     We know that the slave adress for this IMU is 0x68 in
     hexadecimal. For that in the RequestFrom and the
     begin functions we have to put this value.*/

  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  /*We have asked for the 0x3B register. The IMU will send a brust of register.
     The amount of register to read is specify in the requestFrom function.
     In this case we request 6 registers. Each value of acceleration is made out of
     two 8bits registers, low values and high values. For that we request the 6 of them
     and just make then sum of each pair. For that we shift to the left the high values
     register (<<) and make an or (|) operation to add the low values.*/

  Acc_rawX = Wire.read() << 8 | Wire.read(); //each value needs two registres
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();


  /*///This is the part where you need to calculate the angles using Euler equations///*/

  /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw
     values that we have just read by 16384.0 because that is the value that the MPU6050
     datasheet gives us.*/
  /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    to calculate this value in each loop we have done that just once before the setup void.
  */

  /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
      pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
      will calculate the rooth square.*/
  /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;

  /*Now we read the Gyro data in the same way as the Acc data. The adress for the
     gyro data starts at 0x43. We can see this adresses if we look at the register map
     of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for
     the Z axis (YAW).*/

  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true); //Just 4 registers

  Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
    the raw value by 131 because that's the value that the datasheet gives us*/

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX / 131.0;
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY / 131.0;

  /*Now in order to obtain degrees we have to multiply the degree/seconds
    value by the elapsedTime.*/
  /*Finnaly we can apply the final filter where we add the acceleration
    part that afects the angles and ofcourse multiply by 0.98 */

  /*---X axis angle---*/
  Total_angle[0] = 0.90 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.1 * Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.95 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.05 * Acceleration_angle[1];

  /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
//      Serial.print("Total_angle[0]: ");
//      Serial.print(Total_angle[0]);
//      Serial.print("\t");
//      Serial.print("Total_angle[1]: ");
//      Serial.print(Total_angle[1]);
//      Serial.print("\n");

  
    

  /*///////////////////////////P I D///////////////////////////////////*/
  /*Remember that for the balance we will use just one axis. I've choose the x angle
    to implement the PID with. That means that the x axis of the IMU has to be paralel to
    the balance*/

  /*First calculate the error between the desired angle and
    the real measured angle*/
  error =  Total_angle[1] - desired_angle;
  error2 = Total_angle[0] - desired_angle;
  /*Next the proportional value of the PID is just a proportional constant
    multiplied by the error*/

  pid_p = kp * error;
  pid_p_2 = kp_2 * error2;

  /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/
  if (-3 < error < 3)
  {
    pid_i = pid_i + (ki * error);
  }

  if (-3 < error2 < 3)
  {
    pid_i_2 = pid_i_2 + (ki_2 * error2);
  }


  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time.
    Finnaly we multiply the result by the derivate constant*/

  pid_d = kd * ((error - previous_error) / elapsedTime);
  pid_d_2 = kd_2 * ((error2 - previous_error2) / elapsedTime);


  /*The final PID values is the sum of each of this 3 parts*/
  PID = pid_p + pid_i + pid_d;
  PID_2 = pid_p_2 + pid_i_2 + pid_d_2;

  

  

  Serial.print("PID: ");
      Serial.print(PID);
      Serial.print("\t");
      Serial.print("PID_2: ");
      Serial.print(PID_2);
      Serial.print("\n");

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
    tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
    have a value of 2000us the maximum value taht we could sybstract is 1000 and when
    we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
    to reach the maximum 2000us*/
  if (PID < -1000)
  {
    PID = -1000;
  }
  if (PID > 1000)
  {
    PID = 1000;
  }

  if (PID_2 < -1000)
  {
    PID_2 = -1000;
  }
  if (PID_2 > 1000)
  {
    PID_2 = 1000;
  }

  /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/





   speed_Rear_Left = speed_Rear_Left  - PID;
   speed_Front_Left = speed_Front_Left - PID;

   speed_Rear_Right = speed_Rear_Right + PID;
   speed_Front_Right = speed_Front_Right + PID ;



   speed_Front_Right = speed_Front_Right - PID_2;
   speed_Rear_Right = speed_Rear_Right + PID_2;
  
   speed_Front_Left = speed_Front_Left - PID_2;
   speed_Rear_Left = speed_Rear_Left + PID_2;



  /*Once again we map the PWM values to be sure that we won't pass the min
    and max values. Yes, we've already maped the PID values. But for example, for
    throttle value of 1300, if we sum the max PID value we would have 2300us and
    that will mess up the ESC.*/
  //Right
  if (speed_Front_Right < 1000)
  {
    speed_Front_Right = 1000;
  }
  if (speed_Front_Right > 2000)
  {
    speed_Front_Right = 2000;
  }
  //Left
  if (speed_Front_Left < 1000)
  {
    speed_Front_Left = 1000;
  }
  if (speed_Front_Left > 2000)
  {
    speed_Front_Left = 2000;
  }

   if (speed_Rear_Right < 1000)
  {
    speed_Rear_Right = 1000;
  }
  if (speed_Rear_Right > 2000)
  {
    speed_Rear_Right = 2000;
  }
  //Left
  if (speed_Rear_Left < 1000)
  {
    speed_Rear_Left = 1000;
  }
  if (speed_Rear_Left > 2000)
  {
    speed_Rear_Left = 2000;
  }

 
  
//  if(PID<-20 || PID>20){
//     speed_Front_Left =1000;
//     speed_Front_Right = 1000;
//     speed_Rear_Left = 1000;
//     speed_Rear_Right = 1000;
//    }
//   if(PID_2<-20 || PID_2>20){
//     speed_Front_Left =1000;
//     speed_Front_Right = 1000;
//     speed_Rear_Left = 1000;
//     speed_Rear_Right = 1000;
//    }

 



//     Serial.print("speed_Front_Left: ");
//  Serial.print(speed_Front_Left);
//  Serial.print("\t");
//  Serial.print("speed_Front_Right: ");
//  Serial.print(speed_Front_Right);
//  Serial.print("\t");
//  Serial.print("speed_Rear_Left: ");
//  Serial.print(speed_Rear_Left);
//  Serial.print("\t");
//  Serial.print("speed_Rear_Right: ");
//  Serial.print(speed_Rear_Right);
//  Serial.print("\n");
  /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/
  //left_prop.writeMicroseconds(pwmLeft);
  //right_prop.writeMicroseconds(pwmRight);
  previous_error = error; //Remember to store the previous error.
  previous_error2 = error2;

 
  
   
    
  //end gyro


 esc_Front_Left.writeMicroseconds(speed_Front_Left);    // Send the signal to the ESC
  esc_Front_Right.writeMicroseconds(speed_Front_Right);    // Send the signal to the ESC
  esc_Rear_Left.writeMicroseconds(speed_Rear_Left);    // Send the signal to the ESC
  esc_Rear_Right.writeMicroseconds(speed_Rear_Right);    // Send the signal to the ESC

//    Serial.print("speed_Front_Left: ");
//    Serial.print(speed_Front_Left);
//    Serial.print("\t");
//    Serial.print("speed_Front_Right: ");
//    Serial.print(speed_Front_Right);
//    Serial.print("\t");
//    Serial.print("speed_Rear_Left: ");
//    Serial.print(speed_Rear_Left);
//    Serial.print("\t");
//    Serial.print("speed_Rear_Right: ");
//    Serial.print(speed_Rear_Right);
//    Serial.print("\n");



  //
}
