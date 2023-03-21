/*
Ce programme contient le nouvel asservissement, et un systeme de classe pour les moteurs.
*/

#include <AFMotor.h>
#include <TimerOne.h>

#define ENCODEURR 19
#define ENCODEURL 18


double tickperround = 600; // Valeur à changer en fonction du capteur à effet hall

volatile int hallTicksR;
volatile int hallTicksL;

//____________________________________________________LaGrandeClasse___________________________________________

class ClasseMoteur{

  private:
    float _kp;
    float _ki;
    float _kd;
    double _somme_erreur;
    double _delta_erreur;
    double _erreur_precedente;
    double _speedDelta;
    double _MotorPWM;
    double _tickperround;
    
  
  public:

    
    double speedM;
    long ttlTicks;
    double cibleVitesse;
    AF_DCMotor _motor;

    ClasseMoteur(unsigned int tickperround, AF_DCMotor motor){
      
      this->_motor = motor;
      this->_tickperround = tickperround;
      this->speedM = 0;
      this->ttlTicks = 0;
      this->_kp = 120;
      this->_ki = 10;
      this->_kd = 0;
      this->cibleVitesse = 0;
      this->_somme_erreur=0;
      this->_delta_erreur=0;
      this->_erreur_precedente=0;
      this->_MotorPWM=0;
    }


    void routine(int hallTicks){
      MotorAsserv();
      speedM = (hallTicks/_tickperround)/0.02; // Le 0.1 doit correspondre a la fréquence de relevé de speed ligne 20
      ttlTicks += hallTicks;
    }
  
   void MotorAsserv(){ // Fonction qui asservit les moteurs aux variables cibleVitesseL et cibleVitesseR
      // Motor Right
      
      _speedDelta = cibleVitesse - speedM;
      _somme_erreur += _speedDelta;
      _delta_erreur = _speedDelta - _erreur_precedente;
      _erreur_precedente = _speedDelta;
      
      _MotorPWM = _kp * _speedDelta + _ki * _somme_erreur; // + _kd * _delta_erreur;
      
      if (_MotorPWM > 255) _MotorPWM = 255;
      else if (_MotorPWM < 0) _MotorPWM = 0;
      
      _motor.setSpeed(_MotorPWM);
    }

};

//____________________________________________________Fin de LaGrandeClasse___________________________________________


double cibleVitesse;
//Creation des objets moteurs
AF_DCMotor motorR = AF_DCMotor(3, MOTOR34_64KHZ); // Initalisation du moteur branchements sur M1
AF_DCMotor motorL = AF_DCMotor(1, MOTOR12_64KHZ); // Initalisation du moteur branchements sur M1
ClasseMoteur MDROIT = ClasseMoteur(600, motorR);
ClasseMoteur MGAUCHE = ClasseMoteur(600, motorL);


void setup() {
  
  pinMode(ENCODEURR, INPUT_PULLUP);
  pinMode(ENCODEURL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), compteurR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), compteurL, CHANGE);

  Timer1.initialize(20000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerSpeed ); 


  Serial.begin(115200);           // set up Serial library at 115200
  
  cibleVitesse = 0; //___________________________________________Variable de Vitesse !!!!!!_______________________________________________
  ToutDroitCapitaine();
}





void loop() {

  MDROIT._motor.run(FORWARD);
  MGAUCHE._motor.run(FORWARD);

  cibleVitesse = 2.00;
  ToutDroitCapitaine();

  delay(5000);

  cibleVitesse = 0;
  ToutDroitCapitaine();

  delay(1000);

}



void ToutDroitCapitaine(){
  MDROIT.cibleVitesse = cibleVitesse;
  MGAUCHE.cibleVitesse = cibleVitesse;
}

void compteurR(){
  hallTicksR++;  
}

void compteurL(){
  hallTicksL++;
}



void timerSpeed(){

  MDROIT.routine(hallTicksR);
  hallTicksR = 0;
  MGAUCHE.routine(hallTicksL);
  hallTicksL = 0;

  Serial.print("Vitesses: Gauche = ");
  Serial.print(MGAUCHE.speedM);
  Serial.print(" - Droite = ");
  Serial.println(MDROIT.speedM);
}
