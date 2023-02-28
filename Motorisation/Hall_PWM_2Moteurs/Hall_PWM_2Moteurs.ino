/*
Ce programme calcule la vitesse de rotation de deux moteur en arrière plan pendant que le loop exécute n'importe quelle action PWM
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
int hallTicksR;
int hallTicksL;
double speedR = 0;
double speedL = 0;

double cibleVitesseR;
double cibleVitesseL;
double cibleVitesse;

double DeltaR;
double DeltaL;


int rightMotorPWM = 0;
int leftMotorPWM = 0;
//char *var_RightMotor[] = {"FORWARD", "255"};
//char *var_LeftMotor[] = {"FORWARD", "255"};

double tickperround = 600; // Valeur à changer en fonction du capteur à effet hall
 
void setup() {
  pinMode(ENCODEURA, INPUT_PULLUP);
  pinMode(ENCODEURC, INPUT_PULLUP);
  //pinMode(ENCODEURB, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(19), compteurR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), compteurL, CHANGE);

  Timer1.initialize(100000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerSpeed ); 

  Serial.begin(9600);           // set up Serial library at 9600 

  motor_R.run(FORWARD);
  motor_L.run(FORWARD);
  
  cibleVitesse = 2.00;
  ToutDroitCapitaine();
}





void loop() {

  
  DeltaR = cibleVitesseR - speedR;
  if (DeltaR != 0){
    if ((DeltaR > 0 ) && (rightMotorPWM <  255)){
      rightMotorPWM += map(DeltaR, 0, cibleVitesseR, 0.9, 10);
      motor_R.setSpeed(rightMotorPWM);
    }
    if ((DeltaR < 0) && (rightMotorPWM > 0)){
      rightMotorPWM -= 2;
      motor_R.setSpeed(rightMotorPWM);
    }
  }

  DeltaL = cibleVitesseL - speedL;
  if (DeltaL != 0){
    if ((DeltaL > 0) && (leftMotorPWM <  255)){
      leftMotorPWM += map(DeltaL, 0, cibleVitesseL, 0.9, 10);;
      motor_L.setSpeed(leftMotorPWM);
    }
    if ((DeltaL < 0) && (leftMotorPWM > 0)) {
      leftMotorPWM -= 2;
      motor_L.setSpeed(leftMotorPWM);
    }
  }



/*
  while((speedR <= cibleVitesseR) || (speedL <= cibleVitesseL)){
    if (rightMotorPWM < 255) {
      rightMotorPWM += 1;
      Serial.println(rightMotorPWM);
      motor_R.setSpeed(rightMotorPWM);
    }
    if (leftMotorPWM < 255){
      leftMotorPWM += 1;
      Serial.println(leftMotorPWM);
      motor_L.setSpeed(leftMotorPWM);
    }
    delay(10);
  }
*/

 
  
  
  delay(200);
}










void ToutDroitCapitaine(){
  cibleVitesseR = cibleVitesse;
  cibleVitesseL = cibleVitesse;
}

void timerSpeed(){
  //Serial.println(hallTicksR);
  speedR = (hallTicksR/tickperround)/0.1; // Le 0.1 doit correspondre a la fréquence de relevé de speed ligne 20
  speedL = (hallTicksL/tickperround)/0.1; // Le 0.1 doit correspondre a la fréquence de relevé de speed ligne 20
  
  Serial.print("SpeedR = ");
  Serial.print(speedR);
  Serial.print(" | SpeedL =");
  Serial.println(speedL);
  hallTicksR = 0;
  hallTicksL = 0;
}

void compteurL(){
  hallTicksL = hallTicksL + 1;
}

void compteurR(){
  hallTicksR = hallTicksR + 1;
}
