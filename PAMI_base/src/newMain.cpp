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
Pami pami = Pami(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &mesure_pos, &servo, &asserv);

float log_time = 0; // Variable global du temps
int i = 0;

void setup()
{
    pami.setup();
    delay(100);
    // pami.config_start_position();

    pami.set_initial_position(0, 0);
}

void loop()
{
    // pami.allumer_moteur(20);

    // pami.print_encodeur();
    // pami.print_speed();
    // pami.print_position();

    pami.go_to(20, 20, SPEED);
    // if (i == 0)
    // {
    //     pami.reculer(20);
    //     i = 1;
    // }

    pami.print_log();
    // pami.start_match();
}
