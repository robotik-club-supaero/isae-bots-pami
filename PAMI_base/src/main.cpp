/**
 * @file main.cpp
 * @brief Programme principal , a implementer dans le ninja
 */

// #include "esp32-hal-gpio.h"
// #include "esp32-hal.h"
#include <Ninja.h>
#include <Arduino.h>
#include <ESP32Encoder.h>

// Initialise les différents objets
// Ultrason ultrason = Ultrason(ULTRASON_ECHO, ULTRASON_TRIGGER);
Serv servo = Serv(SERVPIN);
Irsensor ir_sensor = Irsensor(IR_SDA_PIN, IR_SCL_PIN, IR_LPN_PIN);
Moteur moteur_r = Moteur(EN_R, IN1_R, IN2_R, INV_MOT_R);
Moteur moteur_l = Moteur(EN_L, IN1_L, IN2_L, INV_MOT_L);
Encodeur encodeur_r = Encodeur(CLK_R, DT_R, INV_ENC_R);
Encodeur encodeur_l = Encodeur(CLK_L, DT_L, INV_ENC_L);

// Le ninja en lui même
Ninja ninja = Ninja(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &servo, &ir_sensor);

float pos_x;
float pos_y;
float angle;

unsigned long newtime;
unsigned long oldtime;
unsigned long newtime_ir;
unsigned long oldtime_ir;

int equipe;

// initialisation de la strat avec les étapes à 0
// ne VRAIMENT pas le bouger d'ici, notamment pas le mettre dans le setup
// c'est une variable extern donc ici on lui assigne juste sa valeur
int etape_globale = 0;

void delay_non_bloquant(int etape_d_appel, unsigned long oldtime)
{
    if ((millis() - oldtime > DELAY_TIME) && (etape_globale == etape_d_appel))
    {
        Serial.println("Etape_global : " + String(etape_globale));
        etape_globale++; // suivant
    }
};

void setup()
{
    Serial.begin(115200); // Initialisation de la communication série
    delay(2000);
    Serial.println("---------- Setup starting ----------");

    // Setup capteur IR
    ir_sensor.setup();
    Serial.println("Setup Done : IR Sensor");

    // Setup servo
    servo.setup();
    Serial.println("Setup Done : Servo");

    // Setup moteur droit & gauche
    moteur_l.setup();
    moteur_r.setup();
    Serial.println("Setup Done : Moteurs");

    // Setup encodeur droit & gauche
    encodeur_r.setup();
    encodeur_l.setup();
    Serial.println("Setup Done : Encodeurs");

    // Led intégrée à l'ESP32 pour blink quand la pompe est activée
    pinMode(PIN_LED, OUTPUT);

    // Pompe
    pinMode(PIN_POMPE, OUTPUT);

    // TIRETTE :
    pinMode(PIN_TIRETTE, INPUT);
    bool etat_tirette = digitalRead(PIN_TIRETTE);
    Serial.print(etat_tirette == 1 ? "Tirette en place \t" : "Tirette enlevée \t");

    // Interrupteur choix équipe
    pinMode(PIN_READEQUIPE, INPUT);
    equipe = digitalRead(PIN_READEQUIPE);
    String color_equipe = (equipe == 1) ? "JAUNE" : "BLEUE";
    Serial.print("Equipe : ");
    Serial.println(color_equipe);

    Serial.println("---------- Setup over ----------\n\n");

    // On attends le début du match, on mettra ensuite toute la stratégie dans la loop qui tourne en continu
    bool tirette_en_place = digitalRead(PIN_TIRETTE);
    while (tirette_en_place)
    {
        delay(10);
        tirette_en_place = digitalRead(PIN_TIRETTE);
    }
    Serial.println("Tirette enlevée, début du match");

    // On remet a 0 les positions car la roue tourne pendant l'upload
    // (car l'esp32 utilise le pin du moteur pendant l'upload)
    encodeur_l.clear_count();
    encodeur_r.clear_count();

    // On commence les timers
    oldtime = millis();
    newtime = millis();

    oldtime_ir = millis();
    newtime_ir = millis();

    // digitalWrite(PIN_LED, 1); // Led ON pour indiquer le setup réussi
    // digitalWrite(PIN_POMPE, HIGH);
    // digitalWrite(PIN_LED, HIGH);
    // delay(60000);
    // digitalWrite(PIN_POMPE, LOW);
    // digitalWrite(PIN_LED, LOW);
}

void loop()
{
    // Pour faire des delais :
    // penser à ne pas updater le temps dans les délais !
    // remplir ici :
    // int etapes_avec_delays[] = {};
    // int taille_tab = sizeof(etapes_avec_delays)/sizeof(int);

    // for (int i = 0; i < taille_tab; i++) {
    //     if (etape_globale != etapes_avec_delays[i]) {
    //         oldtime = newtime;
    //         break;
    //     }
    // }

    oldtime = newtime;
    if (millis() - oldtime > 8 * INTERVAL_ASSERV)
    {
        Serial.print("Etape globale = " + String(etape_globale) + "\t");
        // Serial.println("\t Distance IR = " + String(ir_sensor.ir_minimum_distance));
    }

    // Si on veut de l'IR :
    oldtime_ir = newtime_ir;
    newtime_ir = ir_sensor.loop(oldtime_ir);
    if (ir_sensor.ir_minimum_distance < DISTANCE_MIN && ir_sensor.ir_minimum_distance > 5)
    {
        moteur_l.set_speed(0);
        moteur_r.set_speed(0);
    }
    else
    { // mettre ici la strat }

        float angle = (equipe == 1) ? 90 : -90;
        float angle2 = (equipe == 1) ? 10 : -10;

        newtime = ninja.avancer_asservi(0, 43, oldtime);

        newtime = ninja.tourner_asservi(1, -angle, oldtime);

        newtime = ninja.avancer_asservi(2, 26, oldtime);

        // recallage

        newtime = ninja.avancer_asservi(3, -33, oldtime);

        newtime = ninja.avancer_asservi(4, 5, oldtime);

        // avance vers 2ème caisse

        newtime = ninja.tourner_asservi(5, angle, oldtime);

        newtime = ninja.avancer_asservi(6, 27, oldtime);

        newtime = ninja.tourner_asservi(7, -angle, oldtime);

        // pousse 2ème caisse

        newtime = ninja.avancer_asservi(8, 24, oldtime);

        newtime = ninja.avancer_asservi(9, -26, oldtime);

        newtime = ninja.avancer_asservi(10, 3, oldtime);

        // retourner à la position de départ

        newtime = ninja.tourner_asservi(11, angle, oldtime);
        newtime = ninja.avancer_asservi(12, -73, oldtime);
        newtime = ninja.avancer_asservi(13, 5, oldtime);
        newtime = ninja.tourner_asservi(14, -angle, oldtime);
        newtime = ninja.avancer_asservi(15, 25, oldtime);
        newtime = ninja.tourner_asservi(16, angle, oldtime);
        newtime = ninja.avancer_asservi(17, 27, oldtime);
        newtime = ninja.tourner_asservi(18, -angle, oldtime);

        if (etape_globale == 19)
        {
            servo.blink(TEMPS_BLINK, 0, 90);
        }
    }
