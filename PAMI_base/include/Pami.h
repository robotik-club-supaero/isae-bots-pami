/**
 * @file Pami.h
 * @brief Classe pour controler une PAMI
 *
 */

#ifndef PAMI_H
#define PAMI_H

#include <Arduino.h>
#include <Encodeur.h>
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
    Ultrason *p_ultrason;
    Irsensor *p_ir_sensor;

    int tirette = 1;    // Etat par défaut de la tirette
    int equipe = 0;     // Equipe par défaut (1 = gauche = jaune)
    int num_pami = 1;   // Numéro de la pami
    int int_pami_1 = 0; // Etat interrupteur 1
    int int_pami_2 = 0; // Etat interrupteur 2

    long m_time_log;
    long m_time_match;

    int *p_etape_globale;
    unsigned long p_newtime;

    Pami(int *etape_globale, Moteur *moteur_d, Moteur *moteur_g, Encodeur *encodeur_d, Encodeur *encodeur_g, Serv *servo, Irsensor *ir_sensor = nullptr, Ultrason *ultrason = nullptr);

    void test(int mode);
    void update_setup();
    void delay_non_blocking(unsigned long delay_time, int etape);

    unsigned long avancer_asservi(int etape_d_appel, float consigne_cm, unsigned long oldtime);
    unsigned long tourner_asservi(int etape_d_appel, float consigne_angle, unsigned long oldtime);
    void go_to_asservi(float consigne_x, float consigne_y, unsigned long oldtime);

    // Avancer basiquement sans asserv
    void go_to(float distance_x, float distance_y, int speed = SPEED);
    void avancer(float distance, int speed = SPEED);
    void tourner(float angle_degres, float speed = SPEED);
    void stop();

    void set_speed(float speed);
    void blink_servo(long temps_blink, int angle1, int angle2);

    double get_ultrason_distance();
    double get_IR_distance();

    void print_log();
    void print_encodeur();
    void print_infos_interrupteur();
};

#endif
