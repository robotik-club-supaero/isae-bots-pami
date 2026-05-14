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
Mesure_pos mesure_pos = Mesure_pos(&encodeur_r, &encodeur_l);

int etape_globale = 0;
std::tuple<float, float, unsigned long> resultat;
// La pami en elle même
Pami pami = Pami(&etape_globale, &moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &mesure_pos, &servo, &ir_sensor);

float global_time = 0; // Variable global du temps
float pos_x;
float pos_y;
float angle;

float new_ticks_l, new_ticks_r, newtime;
float old_ticks_l, old_ticks_r, oldtime;

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

    // initialisation de la strat avec les étapes à 0
    oldtime = millis();
    newtime = millis();
    resultat = std::make_tuple(0, 0, 0);

    pami.m_time_match = millis();
    Serial.println("Fin setup");
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

    if (etape_globale != 1)
    {
        new_ticks_l = std::get<0>(resultat);
        new_ticks_r = std::get<1>(resultat);
        newtime = std::get<2>(resultat);
        old_ticks_l = new_ticks_l;
        old_ticks_r = new_ticks_r;
        oldtime = newtime;
    }

    switch (etape_globale)
    {
    case 0:
    {
        float consigne = B_POSITION_1_FINAL_Y + (old_ticks_l + old_ticks_r) / 2;
        resultat = pami.avancer_asservi(0, consigne, consigne, old_ticks_l, old_ticks_r, oldtime);
        break;
    }
    // delay non bloquant
    // case 1:
    // {
    //     if (millis() - oldtime > 1000)
    //     {
    //         etape_globale = 2;
    //         break;
    //     }
    // }
    case 1:
    {
        // tourner
        float consigne_angle = 30;
        resultat = pami.tourner_asservi(1, consigne_angle, old_ticks_l, old_ticks_r, oldtime);
        break;
    }
    case 2:
    {
        // avancer part.2
        float consigne = B_POSITION_1_FINAL_X;
        resultat = pami.avancer_asservi(2, consigne, consigne, old_ticks_l, old_ticks_r, oldtime);
        break;
    }
    }

    // if ((millis() - pami.m_time_match) > START_TIME && (millis() - pami.m_time_match) < ENDTIME)
    // {
    //     if (!start_moving)
    //     {
    //         start_moving = true;
    //         if (pami.num_pami == 1)
    //         {
    //             start_moving = true;
    //             if (pami.equipe == 0) // 0 = bleue, 1 = jaune
    //             {
    //                 Serial.println("Action Match : PAMI 1 BLEUE");
    //                 pami.avancer(B_POSITION_1_FINAL_Y - B_POSITION_1_DEPART_Y);
    //                 pami.tourner(-90);
    //                 pami.avancer(B_POSITION_1_FINAL_X - B_POSITION_1_DEPART_X);
    //             }
    //             else
    //             {
    //                 Serial.println("Action Match : PAMI 1 JAUNE");
    //                 pami.avancer(J_POSITION_1_FINAL_Y - J_POSITION_1_DEPART_Y);
    //                 pami.tourner(90);
    //                 pami.avancer(J_POSITION_1_FINAL_X - J_POSITION_1_DEPART_X);
    //             }
    //         }
    //         else if (pami.num_pami == 2)
    //         {
    //             delay(1000);
    //             start_moving = true;
    //             if (pami.equipe == 0) // 0 = bleue, 1 = jaune
    //             {
    //                 Serial.println("Action Match : PAMI 2 BLEUE");
    //                 pami.avancer(B_POSITION_2_FINAL_Y - B_POSITION_2_DEPART_Y);
    //                 pami.tourner(-90);
    //                 pami.avancer(B_POSITION_2_FINAL_X - B_POSITION_2_DEPART_X);
    //             }
    //             else
    //             {
    //                 Serial.println("Action Match : PAMI 2 JAUNE");
    //                 pami.avancer(B_POSITION_2_FINAL_Y - B_POSITION_2_DEPART_Y);
    //                 pami.tourner(90);
    //                 pami.avancer(B_POSITION_2_FINAL_X - B_POSITION_2_DEPART_X);
    //             }
    //         }
    //         else if (pami.num_pami == 3)
    //         {
    //             delay(2000);
    //             if (pami.equipe == 0) // 0 = bleue, 1 = jaune
    //             {
    //                 Serial.println("Action Match : PAMI 3 BLEUE");
    //                 pami.avancer(B_POSITION_3_FINAL_Y - B_POSITION_3_DEPART_Y);
    //                 pami.tourner(-90);
    //                 pami.avancer(B_POSITION_3_FINAL_X - B_POSITION_3_DEPART_X);
    //             }
    //             else
    //             {
    //                 Serial.println("Action Match : PAMI 3 JAUNE");
    //                 pami.avancer(B_POSITION_3_FINAL_Y - B_POSITION_3_DEPART_Y);
    //                 pami.tourner(90);
    //                 pami.avancer(B_POSITION_3_FINAL_X - B_POSITION_3_DEPART_X);
    //             }
    //         }
    //     }
    // }

    if (millis() - time_last_log >= 1000)
    {
        pami.print_log();
        Serial.println("------------- Time since start match : " + String((millis() - pami.m_time_match) / 1000) + " s -------------");

        time_last_log = millis();
    }
}