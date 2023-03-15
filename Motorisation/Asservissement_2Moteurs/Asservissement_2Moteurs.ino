/*
Ce programme calcule la vitesse de rotation de deux moteur en arrière plan pendant que le loop exécute n'importe quelle action sur les varibales cibleVitesse.
Asservissement en vitesse de rotation sur chaque moteur séparément
Le shield doit etre branche sur la MEGA comme un debile, le moteur est alimenté par le shield, le capteur Hall est en +5V et GND sur la arduino et un seul de ses capteur est utilisé, n'importe lequel, et il doit etre branche sur la pin 19
C'est important que ce soit la PIN 19 et 18 et pas une autre car c'est une PIN dInterrupt.


*/

#include <AFMotor.h>
#include <TimerOne.h>

#define ENCODEURA 19
//#define ENCODEURB 38
#define ENCODEURC 18

AF_DCMotor motor_R(3, MOTOR34_8KHZ); // Initalisation du moteur branchements sur M2
AF_DCMotor motor_L(2, MOTOR12_8KHZ); // Initalisation du moteur branchements sur M1

//Variables d'asservissement
unsigned int kpR = 50; //Coefficient 1 sur moteur droit
unsigned int kpL = 50; //Coefficient 1 sur moteur gauche





volatile int hallTicksR;
volatile int hallTicksL;
double tempTicksR;
double tempTicksL;
double ttlTicksR = 0;
double ttlTicksL = 0;


double speedR = 0;
double speedL = 0;

double cibleVitesseR;
double cibleVitesseL;
double cibleVitesse;

double speedDeltaR;
double speedDeltaL;




double rightMotorPWM = 0;
double leftMotorPWM = 0;
//char *var_RightMotor[] = {"FORWARD", "255"};
//char *var_LeftMotor[] = {"FORWARD", "255"};
boolean AsservTrankil;

double tickperround = 600; // Valeur à changer en fonction du capteur à effet hall
 
void setup() {
  pinMode(ENCODEURA, INPUT_PULLUP);
  pinMode(ENCODEURC, INPUT_PULLUP);
  //pinMode(ENCODEURB, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(19), compteurR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), compteurL, CHANGE);

  Timer1.initialize(50000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerSpeed ); 


  Serial.begin(115200);           // set up Serial library at 115200

  motor_R.run(FORWARD);
  motor_L.run(FORWARD);
  
  cibleVitesse = 1.50; //___________________________________________Variable de Vitesse !!!!!!_______________________________________________
  ToutDroitCapitaine();
}





void loop() {




}










void ToutDroitCapitaine(){
  cibleVitesseR = cibleVitesse;
  cibleVitesseL = cibleVitesse;

  tempTicksR = 0;
  tempTicksL = 0;
  
}

void MotorAsserv(){ // Fonction qui asservit les moteurs aux variables cibleVitesseL et cibleVitesseR

  // Motor Right
  
  speedDeltaR = cibleVitesseR - speedR;
  rightMotorPWM = kpR * speedDeltaR;
  motor_R.setSpeed(rightMotorPWM);
    
  

  // Motor Left

  speedDeltaL = cibleVitesseL - speedL;
  leftMotorPWM = kpL * speedDeltaL;
  motor_L.setSpeed(leftMotorPWM);

  
  
}





void timerSpeed(){
  //Serial.println(hallTicksR);
  if (!(AsservTrankil)){ // Pour ralentir l'asservissement à deux fois le délai de l'interrupt Timer
    MotorAsserv();
    AsservTrankil = true;
  } else {
    AsservTrankil = false;
  }
    
  
  speedR = (hallTicksR/tickperround)/0.05; // Le 0.1 doit correspondre a la fréquence de relevé de speed ligne 20
  speedL = (hallTicksL/tickperround)/0.05; // Le 0.1 doit correspondre a la fréquence de relevé de speed ligne 20

  Serial.print("SpeedL = ");
  Serial.print(speedL);
  Serial.print(" | SpeedR = ");
  Serial.println(speedR);
  
  
  ttlTicksR += hallTicksR;
  tempTicksR += hallTicksR;
  hallTicksR = 0;
  ttlTicksL += hallTicksL;
  tempTicksL += hallTicksL;
  hallTicksL = 0;
}

void compteurL(){
  hallTicksL = hallTicksL + 1;
}

void compteurR(){
  hallTicksR = hallTicksR + 1;
}
