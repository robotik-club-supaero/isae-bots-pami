/**
 * @file ain.cpp
 * @brief Programme principal , a implementer dans la pami
 */

#include <Pami.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Machine_etats.h>

// Initialise les différents objets
// Ultrason ultrason = Ultrason(ULTRASON_ECHO, ULTRASON_TRIGGER);
Serv servo = Serv(SERVPIN);
Irsensor ir_sensor = Irsensor(IR_SDA_PIN, IR_SCL_PIN, IR_LPN_PIN);
Moteur moteur_r = Moteur(EN_R, IN1_R, IN2_R, INV_MOT_R);
Moteur moteur_l = Moteur(EN_L, IN1_L, IN2_L, INV_MOT_L);
Encodeur encodeur_r = Encodeur(CLK_R, DT_R, INV_ENC_R);
Encodeur encodeur_l = Encodeur(CLK_L, DT_L, INV_ENC_L);
Mesure_pos mesure_pos = Mesure_pos(&encodeur_r, &encodeur_l);
Asserv asserv = Asserv(&moteur_r, &moteur_l, &mesure_pos);
// Machine_etats machine_etats = Machine_etats(&asserv, &mesure_pos);

// La pami en elle même
Pami pami = Pami(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &mesure_pos, &servo, &asserv, &ir_sensor); // On n'utilise pas l'ultrason pour le moment

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

    // Setup mesure position
    mesure_pos.setup();
    Serial.println("Setup Done : Mesure de Position");

    // Setup moteur droit & gauche
    moteur_r.setup();
    moteur_l.setup();
    Serial.println("Setup Done : Moteurs");

    // Setup asservissement
    asserv.setup();
    Serial.println("Setup Done : Asservissement");

    pinMode(PIN_TIRETTE, INPUT);
    pinMode(PIN_READEQUIPE, INPUT);
    pinMode(PIN_INT_PAMI_1, INPUT);
    pinMode(PIN_INT_PAMI_2, INPUT);

    pami.config_start_position();

    Serial.println("Setup Done : Tirette & Equipe & PAMI");

    pami.m_time_log = millis();

    Serial.println("\n---------- Setup over ----------\n\n");
    digitalWrite(PIN_LED, LOW);
    delay(500);

    while (digitalRead(PIN_TIRETTE) == 1)
    {
        pami.config_start_position();
        static unsigned long last_diag = 0;
        if (millis() - last_diag > 500)
        {
            pami.print_infos_interrupteur();
            last_diag = millis();
        }
        delay(10);
    }

    // pami.test(7);

    // pami.avancer(100);
    // while (true)
    // {
    //     pami.print_log();
    //     delay(300);
    // }

    pami.tirette = digitalRead(PIN_TIRETTE);
    pami.equipe = digitalRead(PIN_READEQUIPE);
    pami.num_pami = (digitalRead(PIN_INT_PAMI_1) * 2) + digitalRead(PIN_INT_PAMI_2) + 1;

    // On remet a 0 les positions car la roue tourne pendant l'upload (why ?)
    mesure_pos.reinitialise();
    pami.pos_x = 0;
    pami.pos_y = 0;
    pami.angle = 0;
    pami.distance_target = 0;

    pami.m_time_match = millis();
}

bool start_moving = false;

void loop()
{
    static unsigned long time_last_log = 0;
    static unsigned long time_last_sensor = 0;

    pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);

    if (millis() - pami.m_time_match >= ENDTIME)
    {
        pami.stop();
        Serial.println("Temps de match écoulé - Arrêt du robot.");
        while (true)
        {
            pami.set_speed(0);
            pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
        }
    }

    if ((millis() - pami.m_time_match) > START_TIME && (millis() - pami.m_time_match) < ENDTIME)
    {
        if (!start_moving)
        {
            start_moving = true;
            if (pami.num_pami == 1)
            {
                start_moving = true;
                if (pami.equipe == 0) // 0 = bleue, 1 = jaune
                {
                    Serial.println("Action Match : PAMI 1 BLEUE");
                    pami.avancer(1400);
                }
                else
                {
                    Serial.println("Action Match : PAMI 1 JAUNE");
                    pami.avancer(1400);
                }
            }
            else if (pami.num_pami == 2)
            {
                delay(1000);
                start_moving = true;
                if (pami.equipe == 0) // 0 = bleue, 1 = jaune
                {
                    Serial.println("Action Match : PAMI 2 BLEUE");
                    pami.avancer(1000);
                    pami.tourner(-25);
                    pami.avancer(1200);
                }
                else
                {
                    Serial.println("Action Match : PAMI 2 JAUNE");
                    pami.avancer(800);
                    pami.tourner(25);
                    pami.avancer(1000);
                }
            }
            else if (pami.num_pami == 3)
            {
                delay(2000);
                if (pami.equipe == 0) // 0 = bleue, 1 = jaune
                {
                    Serial.println("Action Match : PAMI 3 BLEUE");
                    pami.avancer(1000);
                    pami.tourner(-25);
                    pami.avancer(1200);
                }
                else
                {
                    Serial.println("Action Match : PAMI 3 JAUNE");
                    pami.avancer(800);
                    pami.tourner(25);
                    pami.avancer(1000);
                }
            }
        }
    }

    if (millis() - time_last_log >= 1000)
    {
        pami.print_log();
        Serial.println("------------- Time since start match : " + String((millis() - pami.m_time_match) / 1000) + " s -------------");

        time_last_log = millis();
    }
}