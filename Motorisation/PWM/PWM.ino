/*
Ce programme est un exemple de controle de sens et vitesse en utilisant le shield fournit

*/

#include <AFMotor.h>


AF_DCMotor motor(4); // Initalisation du moteur branchements sur M4
uint8_t i;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
}

void loop() {
 
  motor.run(FORWARD);
  motor.setSpeed(255);

  delay(1000);

  motor.run(RELEASE);

  delay(1000);

  motor.run(BACKWARD);
  motor.setSpeed(150);

  delay(1000);

}
