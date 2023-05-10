#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int val;    // variable to read the value from the analog pin

void setup() {
  Serial.begin(9600);
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object

}

void loop()
{  
  myservo.write(140);                  
  delay(375);    
    
  myservo.write(90);
  delay(1500);

  myservo.write(140);                  
  delay(370);    
    
  myservo.write(90);
  delay(1500);

  myservo.write(140);                  
  delay(365);    
    
  myservo.write(90);
  delay(1500);

  myservo.write(140);                  
  delay(360);    
    
  myservo.write(90);
  delay(6000);

  
//  myservo.write(11);                  
//  delay(400);   
//
//  myservo.write(90);
//  delay(1500);
}
