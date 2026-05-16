/**
 * @file main.cpp
 * @brief Programme principal , a implementer dans la pami
 */

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

// La pami en elle même
Pami pami = Pami(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &servo, &ir_sensor);

unsigned long newtime;
unsigned long oldtime;
unsigned long newtime_ir;
unsigned long oldtime_ir;

int equipe;
float start_time;

// initialisation de la strat avec les étapes à 0
// ne VRAIMENT pas le bouger d'ici, notamment pas le mettre dans le setup
// c'est une variable extern donc ici on lui assigne juste sa valeur
int etape_globale = 0;

void delay_non_bloquant(int etape_d_appel, unsigned long oldtime, float delay_time)
{
    if ((millis() - oldtime > delay_time) && (etape_globale == etape_d_appel))
    {
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

    // LED intégrée à l'ESP32 pour blink quand la pompe est activée
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

    pami.tourner_asservi(0, 360, oldtime);

    // On commence les timers
    oldtime = millis();
    newtime = millis();

    start_time = millis();

    oldtime_ir = millis();
    newtime_ir = millis();
}

void loop()
{
    // Pour faire des delais :
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

    if (millis() - oldtime > 8 * INTERVAL_ASSERV)
    {
        Serial.println("Etape globale = " + String(etape_globale) + "\t");
        // Serial.println("\t Distance IR = " + String(ir_sensor.ir_minimum_distance));
    }

    // Si on veut de l'IR :
    // oldtime_ir = newtime_ir;
    // newtime_ir = ir_sensor.loop(oldtime_ir);
    // if (ir_sensor.ir_minimum_distance<DISTANCE_MIN && ir_sensor.ir_minimum_distance>5){
    //     moteur_l.set_speed(0);
    //     moteur_r.set_speed(0);
    // }
    // else { // mettre ici la strat }

    float angle = (equipe == 1) ? 90 : -90;
    float angleFinal = (equipe == 1) ? 25 : -25;

    // tomber 1er frigo
    newtime = pami.avancer_asservi(0, 43, oldtime);
    newtime = pami.tourner_asservi(1, -angle, oldtime);
    newtime = pami.avancer_asservi(2, 26, oldtime);

    // recallage 1
    newtime = pami.avancer_asservi(3, -33, oldtime);
    newtime = pami.avancer_asservi(4, 6, oldtime);

    // avance vers 2ème caisse
    newtime = pami.tourner_asservi(5, angle, oldtime);
    newtime = pami.avancer_asservi(6, 27, oldtime);
    newtime = pami.tourner_asservi(7, -angle, oldtime);

    // tomber 2eme frigo
    newtime = pami.avancer_asservi(8, 23, oldtime);

    // recallage 2
    newtime = pami.avancer_asservi(9, -31, oldtime);
    newtime = pami.avancer_asservi(10, 6, oldtime);

    // aller vers garde manger
    newtime = pami.avancer_asservi(11, 22, oldtime);
    newtime = pami.tourner_asservi(12, -angle, oldtime);
    newtime = pami.avancer_asservi(13, 28, oldtime);

    // Recuperer pourri 1
    newtime = pami.bouger_servo_non_bloquant(14, 0, oldtime);
    newtime = pami.allumer_pompe(15, oldtime);
    newtime = pami.avancer_asservi(16, -13, oldtime);

    // lacher pourri 1
    newtime = pami.tourner_asservi(17, 180, oldtime);
    newtime = pami.eteindre_pompe(18, oldtime);
    newtime = pami.bouger_servo_non_bloquant(19, 45, oldtime);

    // recallage 3
    newtime = pami.tourner_asservi(20, -angle, oldtime);
    newtime = pami.avancer_asservi(21, -30, oldtime);
    newtime = pami.avancer_asservi(22, 6, oldtime);

    // aller recuperer pourri 2
    newtime = pami.avancer_asservi(23, 19, oldtime);
    newtime = pami.tourner_asservi(24, -angle, oldtime);
    newtime = pami.avancer_asservi(25, 33, oldtime);

    // recuperer pourri 2
    newtime = pami.bouger_servo_non_bloquant(26, 0, oldtime);
    newtime = pami.avancer_asservi(28, -1, oldtime);
    newtime = pami.allumer_pompe(27, oldtime);
    newtime = pami.avancer_asservi(28, -4, oldtime);

    // lacher pourri 2
    newtime = pami.tourner_asservi(29, 180, oldtime);
    newtime = pami.eteindre_pompe(30, oldtime);
    newtime = pami.bouger_servo_non_bloquant(31, 90, oldtime);

    // recallage 4
    newtime = pami.tourner_asservi(32, -angle, oldtime);
    newtime = pami.avancer_asservi(33, -27, oldtime);
    newtime = pami.avancer_asservi(34, 6, oldtime);

    // aller à zone fin
    newtime = pami.tourner_asservi(35, angle, oldtime);
    newtime = pami.avancer_asservi(36, 27, oldtime);
    newtime = pami.tourner_asservi(37, -angle, oldtime);

    // recallage 5 pour la dernière avance
    newtime = pami.avancer_asservi(38, -8, oldtime);
    newtime = pami.avancer_asservi(39, 29, oldtime);
    newtime = pami.tourner_asservi(40, -angleFinal, oldtime);

    // if (etape_globale == 41 || start_time >= ENDTIME)
    // {
    //     servo.blink(TEMPS_BLINK, 0, 90);
    // }
}