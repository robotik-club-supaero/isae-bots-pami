/**
 * @file main.cpp
 * @brief Programme principal , a implementer dans la pami
 */

#include "esp32-hal.h"
#include <Pami.h>
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

int etape_globale;
// La pami en elle même
Pami pami = Pami(&etape_globale,&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &servo, &ir_sensor);

float pos_x;
float pos_y;
float angle;

unsigned long newtime;
unsigned long oldtime;
unsigned long newtime_ir;
unsigned long oldtime_ir;

void delay_non_bloquant(int etape_d_appel, unsigned long oldtime){
    if ((millis()-oldtime > DELAY_TIME ) && (etape_globale == etape_d_appel) ){
        Serial.println("Etape_global : " + String(etape_globale));
        etape_globale++; // suivant
    }
};



void setup()
{
    Serial.begin(115200); // Initialisation de la communication série
    delay(6000);
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


    // LED intégrée à l'ESP32 pour blink quand la configuration est finie
    // pinMode(LED, OUTPUT);
    // digitalWrite(LED, 1); // LED ON pour indiquer le setup réussi


    // TIRETTE :
    pinMode(PIN_TIRETTE, INPUT);
    bool etat_tirette = digitalRead(PIN_TIRETTE);
    Serial.print(etat_tirette==1 ? "Tirette en place \t" : "Tirette enlevée \t");

    // Interrupteur choix équipe
    pinMode(PIN_READEQUIPE, INPUT);
    int read_equipe = digitalRead(PIN_READEQUIPE);
    String color_equipe = (read_equipe == 1) ? "JAUNE" : "BLEUE"; 
    Serial.print("Equipe : ");
    Serial.println(color_equipe);

    // Setup de la position
    if (read_equipe == 1) // JAUNE
    {
        pos_x = J_POSITION_DEPART_X;
        pos_y = J_POSITION_DEPART_Y;
    }
    else
    {
        pos_x = B_POSITION_DEPART_X;
        pos_y = B_POSITION_DEPART_Y;
    }

    Serial.println("---------- Setup over ----------\n\n");

    // On remet a 0 les positions car la roue tourne pendant l'upload 
    // (car l'esp32 utilise le pin du moteur pendant l'upload)
    encodeur_l.clear_count();
    encodeur_r.clear_count();

    // On attends le début du match, on mettra ensuite toute la stratégie dans la loop qui tourne en continu
    bool tirette_en_place = digitalRead(PIN_TIRETTE);
    while(tirette_en_place){
        delay(10);
        tirette_en_place = digitalRead(PIN_TIRETTE);
    }
    Serial.println("Tirette enlevée, début du match");
    
    // On commence les timers
    oldtime=millis();
    newtime=millis();

    oldtime_ir=millis();
    newtime_ir=millis();

    // initialisation de la strat avec les étapes à 0
    etape_globale=0;
}



void loop()
{
    // penser à ne pas updater le temps dans les délais !
    // remplir ici : 
    // int etapes_avec_delays[] = {1,3,5};
    // int taille_tab = sizeof(etapes_avec_delays)/sizeof(int);

    // for (int i = 0; i < taille_tab; i++) {
    //     if (etape_globale != etapes_avec_delays[i]) {
    //         oldtime = newtime;
    //         break;
    //     }
    // }

    

    oldtime = newtime;
    if (millis() - oldtime > 4*INTERVAL_ASSERV) {
        Serial.print("Etape globale = "+String(etape_globale));
        Serial.println("\t Distance IR = " + String(ir_sensor.ir_minimum_distance));
    }

    oldtime_ir = newtime_ir;
    newtime_ir = ir_sensor.loop(oldtime_ir);
    if (ir_sensor.ir_minimum_distance<DISTANCE_MIN && ir_sensor.ir_minimum_distance>5){
        
        moteur_l.set_speed(0);
        moteur_r.set_speed(0);

    }
    else {
        float consigne = 40;
        newtime = pami.avancer_asservi(0,consigne,oldtime);

        // delay_non_bloquant(1, oldtime);

        newtime = pami.tourner_asservi(1,180,oldtime);

        // delay_non_bloquant(3, oldtime);

        newtime = pami.avancer_asservi(2,consigne,oldtime);
        
        // delay_non_bloquant(5, oldtime);

        newtime = pami.tourner_asservi(3,-180,oldtime);
    }
}
