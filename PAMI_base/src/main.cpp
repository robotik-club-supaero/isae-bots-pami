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
Pami pami = Pami(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &mesure_pos, &servo, &asserv); // On n'utilise pas l'ultrason pour le moment

float log_time = 0; // Variable global du temps
int i = 0;

void setup()
{
    pami.setup();
    delay(500);
    // pami.config_start_position();

    pami.set_initial_position(0, 0);

    // On remet a 0 les positions car la roue tourne pendant l'upload (why ?)
    mesure_pos.reinitialise();
}

void loop()
{
    // pami.allumer_moteur(200);

    // Test servo
    // pami.blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);

    // Test moteur ici
    pami.print_encodeur();
    Serial.print("\n");
    pami.print_speed();
    Serial.print("\n");
    pami.print_position();
    Serial.print("\n");
    Serial.print("\n");
    // Test capteur ir
    // int ir_distance = pami.get_IR_distance();
    // Serial.println("Distance IR: " + String(ir_distance) + " mm");
    // if (ir_distance != -1 && ir_distance < DISTANCE_MIN)
    // {
    //     Serial.println("Obstacle détecté ! Distance : " + String(ir_distance) + " mm");
    // }

    // Test de la fonction go_to

    // pami.go_to(20, 20, SPEED);

    // Test avancer ou reculer ou tourner
    if (i == 0)
    {
        pami.avancer(30);
        pami.print_encodeur();
        pami.print_speed();
        i = 1;
    }

    delay(1000);
    // pami.print_log();
    // pami.start_match();
}
