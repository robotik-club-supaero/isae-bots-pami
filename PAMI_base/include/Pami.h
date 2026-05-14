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
    Moteur *m_p_moteur_r;
    Moteur *m_p_moteur_l;
    Encodeur *m_p_encodeur_r;
    Encodeur *m_p_encodeur_l;
    Asserv *m_p_asserv;
    Serv *m_p_servo;
    Mesure_pos *m_p_mesure_pos;
    Ultrason *m_p_ultrason;
    Irsensor *m_p_ir_sensor;

    bool m_match_demarre = false;
    int tirette = 1;    // Etat par défaut de la tirette
    int equipe = 1;     // Equipe par défaut (1 = gauche = jaune)
    int num_pami = 1;   // Numéro de la pami
    int int_pami_1 = 0; // Etat interrupteur 1
    int int_pami_2 = 0; // Etat interrupteur 2

    float pos_x;
    float pos_y;
    float angle;

    float pos_init_x;
    float pos_init_y;
    float pos_target_x;
    float pos_target_y;
    float distance_target = 0;

    long m_time_log;
    long m_time_match;

    Pami(Moteur *p_moteur_d, Moteur *p_moteur_g, Encodeur *p_encodeur_d, Encodeur *p_encodeur_g, Mesure_pos *p_mesure_pos, Serv *p_servo, Asserv *p_asserv, Irsensor *p_ir_sensor = nullptr, Ultrason *p_ultrason = nullptr);

    std::tuple<float, float, unsigned long> avancer_asservi(float tick_distance, float old_ticks_l, float old_ticks_r, unsigned long oldtime);

    void test(int mode);

    void config_start_position();
    void print_log();
    void print_position();
    void print_encodeur();
    void print_speed();
    void print_infos_interrupteur();

    // Avancer basiquement sans asserv
    void go_to(float distance_x, float distance_y, int speed = SPEED);
    void avancer(float distance, int speed = SPEED);
    void tourner(float angle_degres, float speed = SPEED);
    void stop();

    // Avancer avec asservissement mais sans capteur ir
    void go_to_asserv(float pos_target_x, float pos_target_y, int speed = SPEED);
    void avancer_asserv(float distance, int speed = SPEED);
    void reculer_asserv(float distance, int speed = SPEED);
    void tourner_asserv(float angle_degres, float speed = SPEED);

    // Avancer avec asservissement et capteur ir
    bool go_to_with_obstacle(float pos_target_x, float pos_target_y, int speed = SPEED);
    bool avancer_with_obstacle(float distance, int speed = SPEED);

    void set_speed(float speed);
    void blink_servo(long temps_blink, int angle1, int angle2);

    double get_ultrason_distance();
    double get_IR_distance();
};

#endif
