//*****************************************************************************************************************************************

//            Coefficients fixes et à déterminer empiriquement mais Classe pour les moteurs et asservissement en vitesse ok

//            Programmation virages et parcours

//*****************************************************************************************************************************************

#include <AFMotor.h>
#include <TimerOne.h>
#include <Servo.h>
#include <QTRSensors.h>

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
      
      _MotorPWM = _kp * _speedDelta + _ki * _somme_erreur + _kd * _delta_erreur;
      
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

    void datSpeed(float cible){ // EN tour de roue par secondes
      //resetParamAsserv();
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
double rRoues = 34; // en mm

int Start = 34;     // Capteur méanique de Start déclaré connecté broche 34

float deltaS;
float ratio;
float cibleR;
float cibleL;

float nerfG = 1;

double distempR;
double distempL;

//Variables PID suivi de ligne
float Varia;
int sommeErreur = 0;
int lastError;
int position;
int error;
float KP = 0.000214;
float KI = 0;
float KD = 0.000400;
unsigned int sensors[8];

Servo servolver;   // Servo du 360
int delayvolver = 375;

Servo servopousse;

QTRSensors qtr;
uint16_t sensorValues[8];

//Creation des objets moteurs
AF_DCMotor motorR = AF_DCMotor(3, MOTOR34_64KHZ); // Initalisation du moteur branchements sur M1
AF_DCMotor motorL = AF_DCMotor(2, MOTOR12_64KHZ); // Initalisation du moteur branchements sur M1

AF_DCMotor brosse = AF_DCMotor(4, MOTOR34_64KHZ); // Initialisation sur M4 de la brosse

ClasseMoteur MDROIT = ClasseMoteur(600,rRoues, motorR, 200, 30, 0);
ClasseMoteur MGAUCHE = ClasseMoteur(600,rRoues, motorL, 200, 30, 0); // Coefficients d'asservissement à parametrer ! kp,ki,kd

//____________________________________________________Def Fonctions___________________________________________________


void SuiviDligne(float ciblasse){
  
  position = qtr.readLineBlack(sensorValues);
  error = position - 3500;
  sommeErreur += error;
  
    
  Varia = KP * error + KI * sommeErreur + KD * (error - lastError);
  lastError = error;
 

  MDROIT.datSpeed(ciblasse - Varia);
  MGAUCHE.datSpeed(ciblasse + Varia);
  delay(1); //A tester sans 
}

void calibmax(){
  for (uint16_t i = 0; i < 100; i++)
  {
    if (i < 10){
      tourneCont(0.50, "d");
    } 
    else if (i < 30){
      tourneCont(0.50, "g");
    }
    else if (i < 50){
      tourneCont(0.50, "d");
    }
    else if (i < 70){
      tourneCont(0.50, "g");
    }
    else if (i < 90){
      tourneCont(0.50, "d");
    }
    else if (i < 100){
      tourneCont(0.50, "g");
    }
    
    qtr.calibrate();
  }
  MDROIT.datSpeed(0);
  MGAUCHE.datSpeed(0);
}


void ToutDroitCapitaine(float ciblasse, float ticksR, float ticksL){
  
  deltaS = ticksR - ticksL;
  
  cibleR = ciblasse*(1-deltaS*0.001); // Si DeltaS = 500, cibleR = ciblasse*0.5
  cibleL = ciblasse*(1+deltaS*0.001);

  MDROIT.datSpeed(cibleR); 
  MGAUCHE.datSpeed(cibleL);

  if (debug){
    Serial.print("Vitesse cible gauche : ");
    Serial.print(cibleL);
    Serial.print("Vitesse cible droite : ");
    Serial.println(cibleR);
  } else { delay(1); } // Ne fonctionne pas sans ce délai, pas d'idée pourquoi.
}

void capteurLumiere(){
  int lumi = analogRead(A7);
  
}

void avanceDeMm(float distDem, float vitesseDem){
  
  distempR = MDROIT.ttlTicks;
  distempL = MGAUCHE.ttlTicks;

  while(((MDROIT.ttlTicks + MGAUCHE.ttlTicks)/2 - (distempR + distempL)/2) < MDROIT.MMtoTicks(distDem)){
    //delay(100); // Peut etre que DatSpeed trop souvent peut empecher l'asservissement à la bonne vitesse. Ici 10Hz
    ToutDroitCapitaine(vitesseDem, MDROIT.ttlTicks - distempR, MGAUCHE.ttlTicks - distempL);
  }

  ToutDroitCapitaine(0,0,0);
  
}

void tourneCont(float vitessedem, String sens){
  if (sens == "d"){
    MDROIT.motor.run(BACKWARD);   
    MGAUCHE.motor.run(FORWARD);
  } else if (sens == "g"){
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(BACKWARD);
  } else {
    MDROIT.motor.run(RELEASE);   
    MGAUCHE.motor.run(RELEASE);
  }
  MDROIT.datSpeed(vitessedem);
  MGAUCHE.datSpeed(vitessedem);
  
}


void tourne(float vitessedem, String sens){

  distempR = MDROIT.ttlTicks;
  distempL = MGAUCHE.ttlTicks;

  if (sens == "d"){
    MDROIT.motor.run(BACKWARD);   
    MGAUCHE.motor.run(FORWARD);
  } else if (sens == "g"){
    MDROIT.motor.run(FORWARD);   
    MGAUCHE.motor.run(BACKWARD);
  } else {
    MDROIT.motor.run(RELEASE);   
    MGAUCHE.motor.run(RELEASE);
    Serial.println("renseigner le sens 'g' ou 'd' en deuxieme argument");
  }

  while(((MDROIT.ttlTicks + MGAUCHE.ttlTicks)/2 - (distempR + distempL)/2) < MDROIT.MMtoTicks(197)){
    ToutDroitCapitaine(vitessedem, MDROIT.ttlTicks - distempR, MGAUCHE.ttlTicks - distempL);
  }

  ToutDroitCapitaine(0,0,0);
  
}

void leptitgrain(){

  servolver.write(140); 
  delay(delayvolver);   
  servolver.write(90);        
  delayvolver -= 5;
  
}

void timerSpeed(){

  MDROIT.routine(hallTicksR*nerfG);
  hallTicksR = 0;
  MGAUCHE.routine(hallTicksL); //Coeffshlag
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

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8}, 8);
  qtr.setEmitterPin(47);

  pinMode(Start, INPUT);
  digitalWrite (Start, HIGH);

  servolver.attach(9);
  servopousse.attach(10);
  servopousse.write(179);

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

  calibmax();

  while(1){
    SuiviDligne(1.20);
  }
  

}
