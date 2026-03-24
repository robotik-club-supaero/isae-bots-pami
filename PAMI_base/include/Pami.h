#ifndef PAMI_H
#define PAMI_H

#include <Arduino.h>
#include <Mesure_pos.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Asserv.h>
#include <Ultrason.hpp>
#include <Serv.h>
#include <define.h>

class Pami
{
public:
    Moteur *m_p_moteur_d;
    Moteur *m_p_moteur_g;
    Encodeur *m_p_encodeur_d;
    Encodeur *m_p_encodeur_g;
    Asserv *m_p_asserv;
    Serv *m_p_servo;
    Mesure_pos *m_p_mesure_pos;
    Ultrason *m_p_ultrason;
    Irsensor *m_p_ir_sensor;

    int tirette = 1;  // Etat par défaut de la tirette
    int equipe = 1;   // Equipe par défaut (1 = gauche = jaune)
    int num_pami = 1; // Numéro de la pami

    float pos_x;
    float pos_y;
    float angle;

    float pos_init_x;
    float pos_init_y;
    float pos_target_x;
    float pos_target_y;
    float distance_target = 0;

    long m_time_log;

    Pami(Moteur *p_moteur_d, Moteur *p_moteur_g, Encodeur *p_encodeur_d, Encodeur *p_encodeur_g, Mesure_pos *p_mesure_pos, Serv *p_servo, Asserv *p_asserv, Irsensor *p_ir_sensor = nullptr, Ultrason *p_ultrason = nullptr);

    void config_start_position();
    void setup();
    void print_log();
    void print_position();
    void print_encodeur();
    void print_speed();
    void set_initial_position(float pos_initial_x, float pos_initial_y);

    void go_to(float pos_target_x, float pos_target_y, int speed = SPEED);
    void avancer(float distance, int speed = SPEED);
    void reculer(float distance, int speed = SPEED);

    bool go_to_with_obstacle(float pos_target_x, float pos_target_y, int speed = SPEED);
    bool avancer_with_obstacle(float distance, int speed = SPEED);
    bool reculer_with_obstacle(float distance, int speed = SPEED);

    void allumer_moteur(float speed);
    void tourner(float angle_degres); // Chelou
    void blink_servo(long temps_blink, int angle1, int angle2);

    double get_ultrason_distance();
    double get_IR_distance();

    void start_match();
    void end_match();
};

#endif
