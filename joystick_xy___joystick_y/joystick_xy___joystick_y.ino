#include <TimedAction.h> //this library is outdated and it won't work with the newest versions of Arduino. After you downloaded the library - https://playground.arduino.cc/Code/TimedAction/ - you must open the libraries folder, open the TimedAction folder, open the TimedAction.h file and change the 33/34 line (from #include "TimedAction.h" to #include "Arduino.h"). If it is already changed, then you shouldn't have any problems.
#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

Servo esc5;

int val,cnt=0,v[4];
int pos1=90,pos2=90,pos3=90,pos4=90;
int val1,val2,val3,val4;
int timer1,timer2,timer3;
int Ok1=1,Ok2=1,Ok3=1;
int delayy2,delayx1,delayy1;
int k,l;



void readbt() {
    val=Serial.read();
    cnt++;
    v[cnt]=val;
    if(v[1]==1 && cnt==3) {
      cnt=0;
    }
    if(v[1]!=1 && cnt==2) {
      cnt=0;
    }
}

void program() {
  timer1++;
  timer2++;
  timer3++;
}

TimedAction readbtThread = TimedAction(10,readbt);
TimedAction timerThread = TimedAction(1,program);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
  myservo4.attach(6);
  esc5.attach(5,1000,2000);
  esc5.writeMicroseconds(1000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
      while(Serial.available() == 0);
      readbtThread.check();
      timerThread.check();
      if(v[1]==1) {
        Ok1=0;
        val1=v[2];
        val2=v[3];
      }
      if(v[1]==2) {
        Ok2=0;
        val3=v[2];
      }
      if(v[1]==3) {
        Ok3=0;
        val4=v[2];
      }
  }
  joystick1();
  k = joystick2(k);
  l = slider(l);
  esc5.writeMicroseconds(k);

 
  
}

void joystick1() {
  if (Ok1==0) {
    timerThread.check();
    if(v[1]==1 && (v[2]==100 && v[3]==100)) {
      Ok1=1;
    }
    if(val1>100 && pos1<180) {
      delayx1=((200-val1))+15;
      if(timer1>=delayx1) {
        pos1++;
        myservo1.write(pos1);
        timer1=0;
      }
    }
    if(val1<100 && pos1>0) {
      delayx1=val1+15;
      if(timer1>=delayx1) {
        pos1--;
        myservo1.write(pos1);
        timer1=0;
      }
    }
    if(val2>100 && pos2<180) {
      delayy1=((200-val2))+15;
      if(timer2>=delayy1) {
        pos2++;
        myservo2.write(pos2);
        timer2=0;
      }
    }
    if(val2<100 && pos2>0) {
      delayy1=val2+15;
      if(timer2>=delayy1) {
        pos2--;
        myservo2.write(pos2);
        timer2=0;
      }
    }
  }
}

int joystick2(int k) {
  if (Ok2==0) {
    timerThread.check();
    if(v[1]==2 && v[2]==100) {
      Ok2=1;
    }
//    if(val3>70 && pos3<180) {
//      delayy2=((200-val3))+15;
//      if(timer3>=delayy2) {
//        pos3++;
//        //myservo3.write(pos3);
//        k = map(pos3,60,200,1000,2000);
//        Serial.print(pos3);
//        Serial.print(" \n");
//        Serial.print(k);
//        Serial.print(" \n");
//        timer3=0;
//      }
//    }
//    if(val3<70 && pos3>0) {
//      delayy2=val3+15;
//      if(timer3>=delayy2) {
//        pos3--;
//        //myservo3.write(pos3);
//        k = map(pos3,60,200,1000,2000);
//        Serial.print(pos3);
//        Serial.print(" \n");
//        Serial.print(k);
//        Serial.print(" \n");
//        timer3=0;
//      }
//    }
    k = map(val3,60,200,1000,2000);
        Serial.print(val3);
        Serial.print(" \n");
        Serial.print(k);
        Serial.print(" \n");
    

    
  }
  return k;
}

int slider(int l) {
  if(Ok3==0) {
    if(pos4==val4) Ok3=1;
    if (val4>pos4) {
      for(int i=pos4;i<=val4;i++) {
        myservo4.write(i);
        l =map(i,0,180,1000,2000);
        Serial.print(i);
        Serial.print(" \n");
        Serial.print(l);
        Serial.print(" \n");
        
        
        delay(5);
      }
    }
    if(val4<pos4) {
      for(int i=pos4;i>=val4;i--) {
        myservo4.write(i);
        l =map(i,0,180,1000,2000);
        Serial.print(i);
        Serial.print(" \n");
        Serial.print(l);
        Serial.print(" \n");
        
        delay(5);
      }
    }
    pos4=val4;
  }
  return l;
}
