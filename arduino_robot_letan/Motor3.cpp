#include "Arduino.h"
#include "Motor3.h"


Motor3::Motor3(int plus, int minus, int en_a, int en_b) {
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  pinMode(en_a,INPUT_PULLUP);
  pinMode(en_b,INPUT_PULLUP);
  Motor3::plus = plus;
  Motor3::minus = minus;
  Motor3::en_a = en_a;
  Motor3::en_b = en_b;
}

void Motor3::rotate3(int value) {
  if(value>=0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
//    Serial.println("called");
//    Serial.println(plus);
    int out = map(value, 0, 100, 0, 170);
    analogWrite(plus,out);
    // analogWrite(minus,0);
    digitalWrite(minus,LOW);
  }else{
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    int out = map(value, 0, -100, 0, 170);
    analogWrite(minus,out);
    digitalWrite(plus,LOW);
    // analogWrite(plus,0);
    // digitalWrite(minus,HIGH);
  }
}
