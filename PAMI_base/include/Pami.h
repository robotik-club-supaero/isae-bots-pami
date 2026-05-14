#ifndef PAMI_H
#define PAMI_H

#include <Arduino.h>
#include <Mesure_pos.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Ultrason.hpp>
#include <Serv.h>
#include <define.h>

class Pami
{
public:
    Moteur *p_moteur_r;
    Moteur *p_moteur_l;
    Encodeur *p_encodeur_r;
    Encodeur *p_encodeur_l;
    Serv *p_servo;
    Mesure_pos *p_mesure_pos;
    Ultrason *p_ultrason;
    Irsensor *p_ir_sensor;

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

    int *p_etape_globale;

    Pami(int *etape_globale, Moteur *moteur_d, Moteur *moteur_g, Encodeur *encodeur_d, Encodeur *encodeur_g, Mesure_pos *mesure_pos, Serv *servo, Irsensor *ir_sensor = nullptr, Ultrason *ultrason = nullptr);

    void test(int mode);
    std::tuple<float, float, unsigned long> avancer_asservi(int etape_d_appel, float consigne_cm_l, float consigne_cm_r, float old_ticks_l, float old_ticks_r, unsigned long oldtime);
    std::tuple<float, float, unsigned long> tourner_asservi(int etape_d_appel, float consigne_angle, float old_ticks_l, float old_ticks_r, unsigned long oldtime);

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

    void set_speed(float speed);
    void blink_servo(long temps_blink, int angle1, int angle2);

    double get_ultrason_distance();
    double get_IR_distance();
};

#endif
