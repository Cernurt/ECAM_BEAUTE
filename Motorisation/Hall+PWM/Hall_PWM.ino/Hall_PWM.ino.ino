#include <AFMotor.h>
#include <TimerOne.h> // Pour le timer

/// Variables

AF_DCMotor motor(4); // Initalisation du moteur branchements sur M4
uint8_t i;
#define ENCODEURA 19
#define ENCODEURB 20

// prérequis : volatile => pour toute variable qui sera utilise dans les interruptions 

volatile int count =0; // comptage de tick d'encoder  qui sera incrémenté sur interruption " On change " sur l'interruption matériel 0 qui se fait sur le pin 2
volatile double speed =0; // vitesse du moteur 
volatile byte laststate =0;  // etat précédent de l'encodeur 


//_____________________________________________________________________________________________________________________________________
 
void setup() {
  pinMode(ENCODEURA, INPUT_PULLUP);
  pinMode(ENCODEURB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(19),counter, CHANGE); // on crée l'interruption sur changement sur la pin 19 => interruption 4, la routine counter va se faire toute seule sans l'appeler à chaque changement d'état sur le pin 2

  Timer1.initialize(50000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here la fonction timerIsr est une routine qui va se faire automatiquement 20* par secondes sans l'appeler

  
  Serial.begin(9600);           // set up Serial library at 9600 bps

  // turn on motor
  motor.setSpeed(200);
  motor.run(RELEASE);
}

//________________________________________________________________________________________________________________________________________

void loop() {
 
  Serial.print("Tourne en avant - ");
  
  motor.run(FORWARD);
  for (i=0; i<255; i++) {
    motor.setSpeed(i);  
    Serial.print("Nombre de ticks : " );            
    Serial.println(speed);
    delay(10);
 }
 
  for (i=255; i!=0; i--) {
    motor.setSpeed(i);  
    delay(10);
 }
  
  Serial.print("Tourne en arriere - ");

  motor.run(BACKWARD);
  for (i=0; i<255; i++) {
    motor.setSpeed(i);  
    delay(10);
 }
 
  for (i=255; i!=0; i--) {
    motor.setSpeed(i);  
    delay(10);
 }
  

  Serial.print("Arret");
  motor.run(RELEASE);
  delay(1000);

  Serial.println();
}

//___________________________________________________________________________________________________________________________________________________

/// Fonctions


// Encoder counter 0

void counter()
{

  byte state = (PINH & B11111100) | (PINB & B00000011);  // mask pour ne regarder que les changements sur 19 et 20  
  if( state!=laststate)
  {
    (((state&(1<<ENCODEURA))>>ENCODEURA)^((state&(1<<ENCODEURB))>>ENCODEURB))?count--:count++; // Encodage de gray, logique
    laststate=state;
  }
}


// Timer event to calculate speed from counters

void timerIsr()
{
    speed=count; // on a le nombre de tick ( signé ) par unité de temps  = vitesse
    count=0; // On remet le compteur de tick à 0 
}
