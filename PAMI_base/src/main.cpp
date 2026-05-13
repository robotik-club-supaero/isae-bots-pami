/**
 * @file main.cpp
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
Machine_etats machine_etats = Machine_etats(&asserv, &mesure_pos);

// La pami en elle même
Pami pami = Pami(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &mesure_pos, &servo, &asserv, &ir_sensor); // On n'utilise pas l'ultrason pour le moment

float log_time = 0; // Variable global du temps
unsigned long time_start_match = 0;
int i = 0;

void setup()
{
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

    while (digitalRead(PIN_TIRETTE) == 1)
    {
        pami.print_infos_interrupteur();
        delay(500);
    }

    time_start_match = millis();
}

void loop()
{
    if (millis() - time_start_match >= GLOBALTIME) // 100 000 ms = 100s
    {
        Serial.println("Temps de match écoulé - Arrêt du robot.");
        pami.set_speed(0);
        while (true)
        {
            pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
            delay(1000);
        }
    }

    if ((millis() - time_start_match) > START_TIME and (millis() - time_start_match) <= GLOBALTIME)
    {
        float dist = pami.get_IR_distance();
        Serial.print("Distance mesuree : ");
        Serial.print(dist / 10.0);
        Serial.println(" cm");

        if (dist < DISTANCE_MIN && dist > 0.5) // Si un obstacle est détecté à moins de 20 cm
        {
            Serial.println("Obstacle détecté ! Arrêt du robot.");
            pami.set_speed(0);
        }
        else
        {
            pami.set_speed(SPEED);
        }
        delay(200);
    }

    // pami.start_match();

    pami.print_log();
    Serial.println("------------- Time since start match : " + String((millis() - time_start_match) / 1000) + " s -------------");
    delay(1000);
}
