/*******************************/
/* OptiMouse, Benoît ROUSSEAU  */
/* - dialoguer avec un ADS2051 */
/* récupérer sur une souris    */
/* optique         */
/*******************************/

#define BROCHE_HORLOGE 2
#define BROCHE_DONNEES 3
#define _BV(bit) (1 << (bit))

byte LectureRegistre (byte adresse)
{
  int i = 7;
  byte retour = 0;

  pinMode (BROCHE_DONNEES, OUTPUT);
  for (; i>=0; i--)
  {

     digitalWrite (BROCHE_HORLOGE, LOW);
     digitalWrite (BROCHE_DONNEES, adresse & (1 << i));
     digitalWrite (BROCHE_HORLOGE, HIGH);
  }

  pinMode (BROCHE_DONNEES, INPUT);
  delayMicroseconds(100);

  for (i=7; i>=0; i--)
  {
    digitalWrite (BROCHE_HORLOGE, LOW);
    digitalWrite (BROCHE_HORLOGE, HIGH);
    retour |= (digitalRead (BROCHE_DONNEES) << i);
  }
  delayMicroseconds(100);

  return retour;
}

void EcritureRegistre (byte adresse, byte donnee)
{
  int i = 7;

  adresse |= 0x80;

  pinMode (BROCHE_DONNEES, OUTPUT);
  for (; i>=0; i--)
  {

     digitalWrite (BROCHE_HORLOGE, LOW);
     digitalWrite (BROCHE_DONNEES, adresse & (1 << i));
     digitalWrite (BROCHE_HORLOGE, HIGH);
  }

  for (i=7; i>=0; i--)
  {
    digitalWrite (BROCHE_HORLOGE, LOW);
     digitalWrite (BROCHE_DONNEES, donnee & (1 << i));
     digitalWrite (BROCHE_HORLOGE, HIGH);
  }

}

void EnvoiImage()
{
  byte val;
  byte adr;
  Serial.print (">IMG:");

  EcritureRegistre (0x0a, 0x09);
  for (int i=0; i<256; i++)
  {
    do {
  adr = LectureRegistre (0x0d);
  val = LectureRegistre (0x0c);
    } while (val & 0x80);
//    Serial.print (adr, HEX);
//    Serial.print ('>', BYTE);
    Serial.print (val, HEX);
//    Serial.print (13, BYTE);
  }

  Serial.println ();
  EcritureRegistre (0x0a, 0x00);
}

void setup()
{
  pinMode (BROCHE_HORLOGE, OUTPUT);
  pinMode (BROCHE_DONNEES, INPUT);
  Serial.begin(19200);
}

void loop ()
{
  if (LectureRegistre (0x02))
  {
    Serial.print ('>');
    Serial.print (LectureRegistre (0x00), DEC);
    Serial.print ('-');
    Serial.print (LectureRegistre (0x01), DEC);
    Serial.print ('-');
    Serial.print (LectureRegistre (0x03), DEC);
    Serial.print ('-');
    Serial.print (LectureRegistre (0x04), DEC);
    Serial.println ();
  }
  if (Serial.available())
  {
    Serial.read();
    EnvoiImage();
  }
}
 
