#include <AFMotor.h>
#include <TimerOne.h>

#define ENCODEURA 19
#define ENCODEURB 38

AF_DCMotor motor(4); // Initalisation du moteur branchements sur M4
uint8_t i;
int hallticks;
double speed = 0;

double tickperround = 1400; // Valeur à changer en fonction du capteur à effet hall
 
void setup() {
  pinMode(ENCODEURA, INPUT_PULLUP);
  pinMode(ENCODEURB, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(19), test, CHANGE);

  Timer1.initialize(100000); // On défini le timeur : 50000 microseconds ( 0.05 sec - or 20Hz )
  Timer1.attachInterrupt( timerSpeed ); 


  
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Et c'est partiiii !");

  // turn on motor
  motor.setSpeed(200);
 
  motor.run(RELEASE);
}

void loop() {
 
  Serial.print("Tourne en avant - ");
  
  motor.run(FORWARD);
  motor.setSpeed(110);


  
  delay(5000);
  Serial.println("-------------Debut tour-----------------");
  delay(1000);
  Serial.println("---------------Fin tour------------------");
  motor.run(RELEASE);
  delay(10000);

}

void timerSpeed(){
  //Serial.println(hallticks);
  speed = (hallticks/tickperround)/0.1;
  Serial.print("Speed = ");
  Serial.println(speed);
  
  hallticks = 0;
}

void test(){
  hallticks = hallticks + 1;
}
