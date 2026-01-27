/**
 * @file main.cpp
 * @brief Programme principal , a implementer dans la pami
 * Pour faire du debug , decommenter les lignes suivient d'un #DEBUG
 *
 */
#include <Moteur.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Encodeur.h>
#include <Mesure_pos.h>
#include <Serv.h>
#include <define.h>
#include <Asserv.h>
#include <Ultrason.hpp>
#include <Machine_etats.h>

#define IR_PIN 4 ; //  ne sert à rien pour le moment , sert pour l'interruption

// Initialise les différents objets :
Ultrason ultrason = Ultrason(ULTRASON_ECHO, ULTRASON_TRIGGER);
Moteur moteur_d = Moteur(EN_R, IN1_R, IN2_R);
Moteur moteur_g = Moteur(EN_L, IN1_L, IN2_L);
Encodeur encoder_R = Encodeur(CLK_R, DT_R);
Encodeur encoder_L = Encodeur(CLK_L, DT_L);
Mesure_pos mesure_pos = Mesure_pos(&encoder_R, &encoder_L);
Asserv asserv = Asserv(&moteur_d, &moteur_g, &mesure_pos);
// Serv servo = Serv(SERVPIN);
Machine_etats machine_etats = Machine_etats(&asserv, &mesure_pos, &ultrason);


long m_time_log = 0; // Variable de temps ou on stocke le temps actuel

void setup()
{

  ultrason.setup();
  Serial.println("Ultrason setup");

  mesure_pos.setup();
  Serial.println("Mesure de Position setup");

  Serial.begin(115200); // Initialisation de la communication série

  moteur_g.setup(); // Initialisation des moteurs
  moteur_d.setup();
  Serial.println("moteur setup");

  // Serial.println("Test Moteurs");
  // moteur_g.set_speed(-200); // TODO : regler la vitesse pour tester la vitesse max
  // moteur_d.set_speed(-200);

  asserv.setup();
  Serial.println("asserv setup");

  machine_etats.setup();
  Serial.println("machine etats setup");

  m_time_log = millis();

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1); // LED ON pour indiquer le setup réussi
}

void loop()
{

  ultrason.loop();
  mesure_pos.loop();
  machine_etats.loop();

  if (m_time_log + 500 < millis()) // Log toutes les secondes
  {
    Serial.print("Distance: ");
    Serial.println(ultrason.m_distance);
    Serial.print("Vitesse L :");
    Serial.print(mesure_pos.vitesse_l);
    Serial.print(" Vitesse R :");
    Serial.print(mesure_pos.vitesse_r);
    Serial.print(" Pos X :");
    Serial.print(mesure_pos.position_x);
    Serial.print(" Pos Y :");
    Serial.print(mesure_pos.position_y);
    Serial.print(" Theta :");
    Serial.print(mesure_pos.position_theta);
    Serial.print(" Distance Ultrason :");
    Serial.print(ultrason.m_distance);
    Serial.print(" Etat :");
    Serial.println(machine_etats.etat);
    m_time_log = millis();
  }
}
