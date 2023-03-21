/*
Ce programme calcule la vitesse de rotation d'un moteur en arrière plan pendant que le loop exécute n'importe quelle action PWM
Le shield doit etre branche sur la MEGA comme un debile, le moteur est alimenté par le shield, le capteur Hall est en +5V et GND sur la arduino et un seul de ses capteur est utilisé, n'importe lequel, et il doit etre branche sur la pin 19
C'est important que ce soit la PIN 19 et pas une autre car c'est une PIN dInterrupt.


*/

#include <AFMotor.h>
#include <TimerOne.h>

#define ENCODEURA 19
//#define ENCODEURB 38

AF_DCMotor motor(4); // Initalisation du moteur branchements sur M4
uint8_t i;
int hallticks;
double speed = 0;

double tickperround = 1400; // Valeur à changer en fonction du capteur à effet hall
 
void setup() {
  pinMode(ENCODEURA, INPUT_PULLUP);
  //pinMode(ENCODEURB, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(19), compteur, CHANGE);

  Timer1.initialize(100000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerSpeed ); 

  Serial.begin(9600);           // set up Serial library at 9600 bps
}

void loop() {
 
  motor.run(FORWARD);
  motor.setSpeed(255);

}



void timerSpeed(){
  //Serial.println(hallticks);
  speed = (hallticks/tickperround)/0.1; // Le 0.1 doit correspondre a la fréquence de relevé de speed ligne 20
  Serial.print("Speed = ");
  Serial.println(speed);
  hallticks = 0;
}

void compteur(){
  hallticks = hallticks + 1;
}
