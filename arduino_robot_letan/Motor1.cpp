#include "Arduino.h"
#include "Motor1.h"


Motor1::Motor1(int plus, int minus, int en_a, int en_b) {
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  pinMode(en_a,INPUT_PULLUP);
  pinMode(en_b,INPUT_PULLUP);
  Motor1::plus = plus;
  Motor1::minus = minus;
  Motor1::en_a = en_a;
  Motor1::en_b = en_b;
}

void Motor1::rotate1(int value) {
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
