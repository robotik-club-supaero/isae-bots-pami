#ifndef NINJA_H
#define NINJA_H

#include <Arduino.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Serv.h>
#include <Encodeur.h>
#include <define.h>

class Ninja
{
public:
    Moteur *moteur_r;
    Moteur *moteur_l;
    Encodeur *encodeur_r;
    Encodeur *encodeur_l;
    Serv *servo;
    Irsensor *ir_sensor;

    int tirette = 1; // Etat par défaut de la tirette
    int equipe = 1;  // Equipe par défaut (1 = gauche = jaune)

    float pos_x;
    float pos_y;
    float angle;

    float pos_init_x;
    float pos_init_y;

    int etape_globale;

    Ninja(Moteur *p_moteur_d, Moteur *p_moteur_g, Encodeur *p_encodeur_d, Encodeur *p_encodeur_g, Serv *p_servo, Irsensor *p_ir_sensor = nullptr);

    void test(int mode);
    unsigned long avancer_asservi(int ordre_d_appel, float consigne, unsigned long oldtime);
    unsigned long tourner_asservi(int ordre_d_appel, float consigne_angle, unsigned long oldtime);

    void print_encodeur(unsigned long oldtime);

    void blink_servo(long temps_blink, int angle1, int angle2);

    double get_IR_distance(unsigned long oldtime);
};

#endif
