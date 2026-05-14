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
    Moteur *moteur_r;
    Moteur *moteur_l;
    Encodeur *encodeur_r;
    Encodeur *encodeur_l;
    // Asserv *m_p_asserv;
    Serv *servo;
    Mesure_pos *mesure_pos;
    Ultrason *ultrason;
    Irsensor *ir_sensor;

    int tirette = 1;  // Etat par défaut de la tirette
    int equipe = 1;   // Equipe par défaut (1 = gauche = jaune)

    float pos_x;
    float pos_y;
    float angle;

    float pos_init_x;
    float pos_init_y;

    // long m_time;
    int *etape_globale;

    Pami(int *p_ordre_d_appel,Moteur *p_moteur_d, Moteur *p_moteur_g, Encodeur *p_encodeur_d, Encodeur *p_encodeur_g, Mesure_pos *p_mesure_pos, Serv *p_servo, Irsensor *p_ir_sensor = nullptr, Ultrason *p_ultrason = nullptr);

    
    void test(int mode);
    std::tuple<float, float, unsigned long> avancer_asservi(int ordre_d_appel,float consigne_l, float consigne_r,float old_ticks_l,float old_ticks_r,unsigned long oldtime);

    void setup();
    void print_log();
    void print_position();
    void print_encodeur();
    void print_speed();

    void go_to(float pos_target_x, float pos_target_y, int speed = SPEED);
    void avancer_asserv(float distance, int speed = SPEED);
    void reculer_asserv(float distance, int speed = SPEED);
    void tourner_asserv(float angle_degres, float speed = SPEED);

    void avancer(float distance, int speed = SPEED);
    void reculer(float distance, int speed = SPEED);
    void tourner(float angle_degres, float speed = SPEED);

    bool go_to_with_obstacle(float pos_target_x, float pos_target_y, int speed = SPEED);
    bool avancer_with_obstacle(float distance, int speed = SPEED);
    bool reculer_with_obstacle(float distance, int speed = SPEED);

    void tout_droit(float speed);
    void blink_servo(long temps_blink, int angle1, int angle2);

    double get_ultrason_distance();
    double get_IR_distance();

    void start_match();
    void end_match();
};

#endif
