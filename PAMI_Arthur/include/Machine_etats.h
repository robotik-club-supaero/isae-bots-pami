
#ifndef MACHINE_ETATS_H
#define MACHINE_ETATS_H

#include <Arduino.h>
#include <vector>
#include <Mesure_pos.h>
#include <Moteur.h>
#include <Irsensor.h>
#include <Asserv.h>
#include "Ultrason.h"
#include "Serv.h"
#include "define.h"

#define K 1
#define dt 10

#define time_sensor 8000

struct Point {
    float x;
    float y;
    Point(float x, float y) : x(x), y(y) {}
};

double distance(const Point& a, const Point& b);
bool isAtTarget(const Point& currentPos, const Point& target, double threshold = 1.);

class Machine_etats
{
    enum Pami_State
    {
        INIT,
        MOVE,
        OBSTACLE,
        STOP,
        END,

    };

public:
    Pami_State etat;
    long m_time;
    long m_time_global;
    long m_time_sensor;

    int tirette = 1; // Etat par défaut de la tirette
    int equipe = 1; // Equipe par défaut (1 = gauche = jaune)
    
    float pos_init_x = J_POSITION_DEPART_X;
    float pos_init_y = J_POSITION_DEPART_Y;
    float pos_target_x = J_POSITION_FINAL_X; 
    float pos_target_y = J_POSITION_FINAL_Y; 
    float distance_target = 0 ;
    
    float pos_x = 0;
    float pos_y = 0;
    float angle = 0;
    int m_minimum_distance = 1000; 

    int checkpoints_loop = 1 ;
    int targetIndex = 0 ;
    std::vector<Point> checkpoints = {
        {20.0, 0.0},  // p1
        {20.0, 20.0},  // p1
        {0.0, 20.0},  // p1
        {0.0, 0.0}   // p4
    };
    Point target = checkpoints[0] ;
    
    Asserv *m_p_asserv;
    Serv *m_p_servo ;

    Mesure_pos *m_p_mesure_pos;
    Ultrason *m_p_ultrason;

    Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Ultrason *p_ultrason);
    void setup();
    void loop();
};

#endif
