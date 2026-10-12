
#include <Serv.h>

Serv::Serv(int pin)
{
  this->servo = Servo(); // Objet de la library Serv.h
  this->pin = pin; // Setup le PIN
}

void Serv::blink(long temps_blink, int angle1, int angle2)
{
  if (millis() - temps_servo > temps_blink)
  {
    if (etat == angle1)
    {
      this->servo.write(angle2);
      etat = angle2;
      Serial.println("Servo moved to angle2");
    }
    else
    {
      this->servo.write(angle1);
      etat = angle1;
      Serial.println("Servo moved to angle1");
    }
    temps_servo = millis();
  }
}

void Serv::setup()
{
  temps_servo = millis();
  servo.attach(pin); // demandé par la l'objet servo de la library

  etat = 90; // Initialise l'etat du servo 
  servo.write(etat);
}

void Serv::loop() // Actuellement fait juste bouger le servo de droite à gauche.
{
  this->blink(1000, 60, 120) ;
}
