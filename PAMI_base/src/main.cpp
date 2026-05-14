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

    if (TEST){
        pami.setup();
        Serial.println("TEST");

        while (true)
        {   
            Serial.print("Droit : ");
            Serial.print(pami.m_p_encodeur_d->mesure());
            Serial.print(" | Gauche : ");
            Serial.println(pami.m_p_encodeur_g->mesure());
            delay(200);
            pami.m_p_moteur_d->stop();
            pami.m_p_moteur_g->stop();
            delay(200);
            pami.tourner(-90);
        }

    }

    pami.setup();
    delay(500);
    pami.config_start_position();
    // pami.set_initial_position(0, 0); //juste pour test manuellement

    // On remet a 0 les positions car la roue tourne pendant l'upload (why ?)
    mesure_pos.reinitialise();
    pami.pos_x = 0;
    pami.pos_y = 0;
    pami.angle = 0;
    pami.distance_target = 0;

    pami.test(7);

    // pami.go_to_asserv(0, 500);

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

    pami.tirette = digitalRead(PIN_TIRETTE);
    pami.equipe = digitalRead(PIN_READEQUIPE);
    pami.num_pami = (digitalRead(PIN_INT_PAMI_1) * 2) + digitalRead(PIN_INT_PAMI_2) + 1;

    pami.m_time_match = millis();
}

bool start_moving = false;

void loop()
{
    
    if (TEST) {return ;}

    static unsigned long time_last_log = 0;
    static unsigned long time_last_sensor = 0;

    if (millis() - pami.m_time_match >= GLOBALTIME)
    {
        pami.set_speed(0);
        Serial.println("Temps de match écoulé - Arrêt du robot.");
        while (true)
        {
            pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
            delay(1000);
        }
    }

    if ((millis() - pami.m_time_match) > START_TIME && (millis() - pami.m_time_match) < GLOBALTIME)
    {
        if (!start_moving)
        {
            if (pami.num_pami == 1)
            {
                start_moving = true;
                if (pami.equipe == 0) // 0 = bleue, 1 = jaune
                {
                    Serial.println("Action Match : PAMI 1 BLEUE");
                    pami.avancer(800, 150);
                }
                else
                {
                    Serial.println("Action Match : PAMI 1 JAUNE");
                    pami.avancer(800, 150);
                }
            }
            else if (pami.num_pami == 2)
            {
                delay(1000);
                start_moving = true;
                if (pami.equipe == 0) // 0 = bleue, 1 = jaune
                {
                    // Serial.println("Action Match : PAMI 2 BLEUE");
                    // pami.avancer(700, 150);
                    // delay(200);
                    // pami.tourner(-60);
                    // delay(200);
                    // pami.avancer(800, 150);
                    while (true)
                    {
                        pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
                        delay(1000);
                    }
                }
                else
                {
                    // Serial.println("Action Match : PAMI 2 JAUNE");
                    // pami.avancer(700, 150);
                    // delay(200);
                    // pami.tourner(60);
                    // delay(200);
                    // pami.avancer(800, 150);
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