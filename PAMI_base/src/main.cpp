/**
 * @file ain.cpp
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

int etape_globale = 0;
float newtime;
float oldtime;

// La pami en elle même
Pami pami = Pami(&etape_globale, &moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &servo, &ir_sensor);

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
    Serial.begin(9600);
    Serial.println("---------- Setup starting ----------");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); // LED ON pour indiquer le début du setup

    // Setup capteur IR
    if (&ir_sensor != nullptr)
    {
        ir_sensor.setup();
        Serial.println("Setup Done : IR Sensor");
    }

    // Setup servo
    servo.setup();
    Serial.println("Setup Done : Servo");

    // Setup moteur droit & gauche
    moteur_r.setup();
    moteur_l.setup();
    Serial.println("Setup Done : Moteurs");

    // Setup encodeur droit & gauche
    encodeur_r.setup();
    encodeur_l.setup();
    Serial.println("Setup Done : Encodeurs");

    pinMode(PIN_TIRETTE, INPUT);
    pinMode(PIN_READEQUIPE, INPUT);
    pinMode(PIN_INT_PAMI_1, INPUT);
    pinMode(PIN_INT_PAMI_2, INPUT);
    pami.print_infos_interrupteur();
    Serial.println("Setup Done : Tirette & Equipe & PAMI");

    Serial.println("\n---------- Setup over ----------\n\n");
    digitalWrite(PIN_LED, LOW);
    delay(500);

    unsigned long last_diag = 0;

    while (digitalRead(PIN_TIRETTE) == 1)
    {
        // pami.update_setup();

        if (millis() - last_diag > 500)
        {
            pami.print_infos_interrupteur();
            last_diag = millis();
        }
        delay(10);
    }

    // On remet a 0 les positions car la roue tourne pendant l'upload (why ?)
    encodeur_l.clear_count();
    encodeur_r.clear_count();

    // pami.test(4);

    // Temps des fonctions non bloquantes
    oldtime = millis();
    newtime = millis();

    // equivalent a newtime ?
    pami.m_time_log = millis();
    pami.m_time_match = millis();
    Serial.println("Fin setup");
}

void loop()
{
    // Temps local pour des logs toutes les secondes
    static unsigned long time_last_log = 0;

    // penser à ne pas updater le temps dans les délais !
    // remplir ici :
    int etapes_avec_delays[] = {1, 3, 5};
    int taille_tab = sizeof(etapes_avec_delays) / sizeof(int);

    for (int i = 0; i < taille_tab; i++)
    {
        if (etape_globale != etapes_avec_delays[i])
        {
            oldtime = newtime;
            break;
        }
    }

    // Condition de fin de match
    if (millis() - pami.m_time_match >= ENDTIME)
    {
        pami.set_speed(0);
        Serial.println("Temps de match écoulé - Arrêt du robot.");
        while (true)
        {
            pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
        }
    }

    // Fonction constamment
    pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);

    // newtime = pami.tourner_asservi(0, 360 * 3, oldtime);

    float dist = pami.get_IR_distance();

    if (dist < DISTANCE_MIN && dist > 0.5) // Si un obstacle est détecté à moins de DISTANCE_MIN cm
    {
        Serial.println("Obstacle détecté ! Arrêt du robot.");
        pami.set_speed(0);
        return;
    }

    // newtime = pami.avancer_asservi(0, 20, oldtime);
    // newtime = pami.tourner_asservi(0, 180, oldtime);

    if ((millis() - pami.m_time_match) > START_TIME && (millis() - pami.m_time_match) < ENDTIME)
    {
        pami.print_infos_interrupteur();
        if (pami.num_pami == 1)
        {
            if (pami.equipe == 0) // 0 = bleue, 1 = jaune
            {
                // newtime = pami.avancer_asservi(0, 20, oldtime);
                newtime = pami.avancer_asservi(0, B_DELTA_1_X, oldtime);
                delay_non_bloquant(1, oldtime);
                newtime = pami.tourner_asservi(2, -90, oldtime);
                delay_non_bloquant(3, oldtime);
                newtime = pami.avancer_asservi(4, B_DELTA_1_Y, oldtime);
            }
            else
            {
                newtime = pami.avancer_asservi(0, J_DELTA_1_X, oldtime);
                delay_non_bloquant(1, oldtime);
                newtime = pami.tourner_asservi(2, 90, oldtime);
                delay_non_bloquant(3, oldtime);
                newtime = pami.avancer_asservi(4, J_DELTA_1_Y, oldtime);
            }
        }
        else if (pami.num_pami == 2)
        {
            if (pami.equipe == 0) // 0 = bleue, 1 = jaune
            {
                newtime = pami.avancer_asservi(0, B_DELTA_2_X, oldtime);
                delay_non_bloquant(1, oldtime);
                newtime = pami.tourner_asservi(2, -90, oldtime);
                delay_non_bloquant(3, oldtime);
                newtime = pami.avancer_asservi(4, B_DELTA_2_Y, oldtime);
            }
            else
            {
                newtime = pami.avancer_asservi(0, J_DELTA_2_X, oldtime);
                delay_non_bloquant(1, oldtime);
                newtime = pami.tourner_asservi(2, 90, oldtime);
                delay_non_bloquant(3, oldtime);
                newtime = pami.avancer_asservi(4, J_DELTA_2_Y, oldtime);
            }
        }
        else if (pami.num_pami == 3)
        {
            if (pami.equipe == 0) // 0 = bleue, 1 = jaune
            {
                newtime = pami.avancer_asservi(0, B_DELTA_3_X, oldtime);
                delay_non_bloquant(1, oldtime);
                newtime = pami.tourner_asservi(2, 90, oldtime);
                delay_non_bloquant(3, oldtime);
                newtime = pami.avancer_asservi(4, B_DELTA_3_Y, oldtime);
            }
            else
            {
                newtime = pami.avancer_asservi(0, J_DELTA_3_X, oldtime);
                delay_non_bloquant(1, oldtime);
                newtime = pami.tourner_asservi(2, -90, oldtime);
                delay_non_bloquant(3, oldtime);
                newtime = pami.avancer_asservi(4, J_DELTA_3_Y, oldtime);
            }
        }
    }

    if (millis() - time_last_log >= 1000)
    {
        pami.print_log();
        Serial.println("------------- Time since match started : " + String((millis() - pami.m_time_match) / 1000) + " s -------------");

        time_last_log = millis();
    }
}