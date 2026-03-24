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
Irsensor ir_sensor = Irsensor(IR_SDA_PIN, IR_SCL_PIN, IR_LPN_PIN);
Moteur moteur_d = Moteur(EN_R, IN1_R, IN2_R);
Moteur moteur_g = Moteur(EN_L, IN1_L, IN2_L);
Encodeur encodeur_d = Encodeur(CLK_R, DT_R);
Encodeur encodeur_g = Encodeur(CLK_L, DT_L);
Mesure_pos mesure_pos = Mesure_pos(&encodeur_d, &encodeur_g);
Asserv asserv = Asserv(&moteur_d, &moteur_g, &mesure_pos);
Serv servo = Serv(SERVPIN);
Machine_etats machine_etats = Machine_etats(&asserv, &mesure_pos);

// La pami en elle même
Pami pami = Pami(&moteur_d, &moteur_g, &encodeur_d, &encodeur_g, &mesure_pos, &servo, &asserv, &ir_sensor);

float log_time = 0; // Variable global du temps

void setup()
{
    pami.setup();
    delay(100);
    // pami.config_start_position();

    pami.set_initial_position(0, 0);
}

int i = 0;

void loop()
{
    pami.print_log();
    // pami.allumer_moteur(0);
    // pami.go_to_with_obstacle(10, 7, SPEED);
    // delay(250);

    // pami.start_match();
}
