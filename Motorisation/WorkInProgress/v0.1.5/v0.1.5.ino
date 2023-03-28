//*****************************************************************************************************************************************

//            Coefficients fixes et à déterminer empiriquement mais Classe pour les moteurs et asservissement en vitesse ok

//            J'essaie de faire l'asservissement pour que le robot roule droit

//*****************************************************************************************************************************************

#include <AFMotor.h>
#include <TimerOne.h>

#define ENCODEURR 19
#define ENCODEURL 18

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
    float _rRoues;
    
  
  public:

    
    double speedM;
    long ttlTicks;
    double cibleVitesse;
    AF_DCMotor motor;

    ClasseMoteur(unsigned int tickperround,float rRoues, AF_DCMotor motor,float kp, float ki, float kd){
      
      this->motor = motor;
      this->_rRoues = rRoues;
      this->_tickperround = tickperround;
      this->speedM = 0;
      this->ttlTicks = 0;
      this->_kp = kp;
      this->_ki = ki;
      this->_kd = kd;
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
      
      motor.setSpeed(_MotorPWM);
    }

    double distanceParcourue(){
      return ((ttlTicks/_tickperround)*2*3.1416*_rRoues);
    }

    double MMtoTicks(float temp){
      return ((temp/(2*3.1416*_rRoues))*_tickperround);
    }

    double TickstoMM(float temp){
      return ((temp/_tickperround)*2*3.1416*_rRoues);
    }

    void datSpeed(float cible){
      resetParamAsserv();
      cibleVitesse = cible;
    }

    void resetParamAsserv(){
      _somme_erreur=0;
      _delta_erreur=0;
      _erreur_precedente=0;
    }

};

//____________________________________________________Fin de LaGrandeClasse___________________________________________

//___________________________________________________Création objets/variables_________________________________________________________
bool debug=true;

double tickperround = 600; // Valeur à changer en fonction du capteur à effet hall
volatile int hallTicksR;
volatile int hallTicksL;
double rRoues = 33; // en mm

int Start = 34;     // Capteur méanique de Start déclaré connecté broche 34

float coefbiMot;
float deltaS;
float ratio;
float cibleR;
float cibleL;

double distempR;
double distempL;

//Creation des objets moteurs
AF_DCMotor motorR = AF_DCMotor(3, MOTOR34_64KHZ); // Initalisation du moteur branchements sur M1
AF_DCMotor motorL = AF_DCMotor(2, MOTOR12_64KHZ); // Initalisation du moteur branchements sur M1
ClasseMoteur MDROIT = ClasseMoteur(600,rRoues, motorR, 150, 10, 0);
ClasseMoteur MGAUCHE = ClasseMoteur(600,rRoues, motorL, 150, 10, 0); // Coefficients d'asservissement à parametrer ! kp,ki,kd

//____________________________________________________Def Fonctions___________________________________________________

void ToutDroitCapitaine(float ciblasse, float ticksR, float ticksL){
  
  deltaS = ticksR - ticksL;
  ratio = map(deltaS, -1000, 1000, -0.30, 0.30);
  cibleR = ciblasse*(1-ratio);
  cibleL = ciblasse*(1+ratio);

  MDROIT.datSpeed(cibleR);
  MGAUCHE.datSpeed(cibleL);
}

void avanceDeMm(float distDem, float vitesseDem){
  
  distempR = MDROIT.ttlTicks;
  distempL = MGAUCHE.ttlTicks;

  while(((MDROIT.ttlTicks + MGAUCHE.ttlTicks)/2 - (distempR + distempL)/2) < MDROIT.MMtoTicks(distDem)){
    ToutDroitCapitaine(vitesseDem, MDROIT.ttlTicks - distempR, MGAUCHE.ttlTicks - distempL);
  }

  ToutDroitCapitaine(0,0,0);
  
}

void timerSpeed(){

  MDROIT.routine(hallTicksR);
  hallTicksR = 0;
  MGAUCHE.routine(hallTicksL);
  hallTicksL = 0;

  if (debug){
    Serial.print("Vitesses: Gauche = ");
    Serial.print(MGAUCHE.speedM);
    Serial.print(" - Droite = ");
    Serial.println(MDROIT.speedM);
  }
}

void compteurR(){hallTicksR++;}
void compteurL(){hallTicksL++;}

//____________________________________________________Debut programme_________________________________________________________



void setup() {
  Serial.begin(115200);           // set up Serial library at 115200

  pinMode(Start, INPUT);
  digitalWrite (Start, HIGH);


  Serial.println("Attente d'appui sur Start");

  if (digitalRead(Start)==HIGH){         //
    while(digitalRead(Start)==HIGH);     //    Lancement si l'interrupteur de start change d'état
  } else if (digitalRead(Start)==LOW){   //
    while(digitalRead(Start)==LOW);      //
  }

  
  pinMode(ENCODEURR, INPUT_PULLUP);
  pinMode(ENCODEURL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), compteurR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), compteurL, CHANGE);

  Timer1.initialize(20000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerSpeed ); 

 

  MDROIT.motor.run(FORWARD);     // ^
  MGAUCHE.motor.run(FORWARD);    // | Exemple de commande des moteurs
  avanceDeMm(0, 1);              // v   (distanceEnMm, vitesse en tr/sec)
}


void loop() {

  MDROIT.motor.run(FORWARD);   
  MGAUCHE.motor.run(FORWARD);
  avanceDeMm(3000, 2);

  Serial.println(MDROIT.ttlTicks);
  Serial.println(MGAUCHE.ttlTicks);

  Serial.println(MDROIT.distanceParcourue());
  delay(6000);

  MDROIT.motor.run(BACKWARD);   
  MGAUCHE.motor.run(BACKWARD);
  avanceDeMm(300, 2);

  delay(3000);

  

}
