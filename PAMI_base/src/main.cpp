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
unsigned long newtime;
unsigned long oldtime;

// La pami en elle même
Pami pami = Pami(&etape_globale, &moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &servo, &ir_sensor);

void setup()
{
    Serial.begin(9600);
    Serial.println("---------- Setup starting ----------");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); // LED ON pour indiquer le début du setup

    pinMode(PIN_TIRETTE, INPUT);
    pinMode(PIN_READEQUIPE, INPUT);
    pinMode(PIN_INT_PAMI_1, INPUT);
    pinMode(PIN_INT_PAMI_2, INPUT);
    Serial.println("Setup Done : Tirette & Equipe & PAMI");

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

    pami.m_time_log = millis();

    Serial.println("\n---------- Setup over ----------\n\n");
    digitalWrite(PIN_LED, LOW);
    delay(500);

    while (digitalRead(PIN_TIRETTE) == 1)
    {
        pami.update_setup();

        static unsigned long last_diag = 0;
        if (millis() - last_diag > 500)
        {
            pami.print_infos_interrupteur();
            last_diag = millis();
        }
        delay(10);
    }

    // pami.test(7);

    // On remet a 0 les positions car la roue tourne pendant l'upload (why ?)
    encodeur_l.clear_count();
    encodeur_r.clear_count();

    // Temps des fonctions non bloquantes
    oldtime = millis();
    newtime = millis();

    pami.m_time_match = millis();
    Serial.println("Fin setup");
}

void loop()
{
    static unsigned long time_last_log = 0;

    pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);

    if (millis() - pami.m_time_match >= ENDTIME)
    {
        pami.set_speed(0);
        Serial.println("Temps de match écoulé - Arrêt du robot.");
        while (true)
        {
            pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
        }
    }

    if ((millis() - pami.m_time_match) > START_TIME && (millis() - pami.m_time_match) < ENDTIME)
    {
        pami.print_infos_interrupteur();
        if (pami.num_pami == 1)
        {
            if (pami.equipe == 0) // 0 = bleue, 1 = jaune
            {
                newtime = pami.avancer_asservi(0, B_POSITION_1_FINAL_Y, oldtime);
                newtime = pami.tourner_asservi(1, -90, oldtime);
                newtime = pami.avancer_asservi(2, B_POSITION_1_FINAL_X, oldtime);
            }
            else
            {
                newtime = pami.avancer_asservi(0, J_POSITION_1_FINAL_Y, oldtime);
                newtime = pami.tourner_asservi(1, 90, oldtime);
                newtime = pami.avancer_asservi(2, J_POSITION_1_FINAL_X, oldtime);
            }
        }
        else if (pami.num_pami == 2)
        {
            if (pami.equipe == 0) // 0 = bleue, 1 = jaune
            {
                newtime = pami.avancer_asservi(0, B_POSITION_2_FINAL_Y, oldtime);
                newtime = pami.tourner_asservi(1, -90, oldtime);
                newtime = pami.avancer_asservi(2, B_POSITION_2_FINAL_X, oldtime);
            }
            else
            {
                newtime = pami.avancer_asservi(0, J_POSITION_2_FINAL_Y, oldtime);
                newtime = pami.tourner_asservi(1, 90, oldtime);
                newtime = pami.avancer_asservi(2, J_POSITION_2_FINAL_X, oldtime);
            }
        }
        else if (pami.num_pami == 3)
        {
            if (pami.equipe == 0) // 0 = bleue, 1 = jaune
            {
                newtime = pami.avancer_asservi(0, B_POSITION_3_FINAL_Y, oldtime);
                newtime = pami.tourner_asservi(1, -90, oldtime);
                newtime = pami.avancer_asservi(2, B_POSITION_3_FINAL_X, oldtime);
            }
            else
            {
                newtime = pami.avancer_asservi(0, J_POSITION_3_FINAL_Y, oldtime);
                newtime = pami.tourner_asservi(1, 90, oldtime);
                newtime = pami.avancer_asservi(2, J_POSITION_3_FINAL_X, oldtime);
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