/**
 * @file Mesure_pos.h
 * @brief Mesure de la position et de la vitesse du robot voir readme pour plus de détail
 */

#ifndef MESURE_POS_H
#define MESURE_POS_H
#include <Arduino.h>
#include <define.h>
#include <Encodeur.h>

class Mesure_pos
{

private:
    Encodeur *m_p_encoder_R;
    Encodeur *m_p_encoder_L;
    /**
     * Facteurs proportionnels entre encodeur et roue ( a determiner empiriquement )
     * Permet de passer de la mesure de l'encodeur à la distance parcourue par la roue et à l'angle de rotation
     */
    float K_angle = 0.0069; // TODO : regler les facteurs si on change les roues & encodeurs
    float K_r = 0.094;      // TODO : regler les facteurs si on change les roues & encodeurs
    float K_l = 0.102;      // TODO : regler les facteurs si on change les roues & encodeurs

    /**
     * temps entre deux mesures
     */
    long dt;

    float position_x_prec;
    float position_y_prec;

    /**
     * temps mis a jour à chaque boucle
     * 2 variables permet d'éviter les bruits/erreurs lors de la conversion millis & micros
     * On utilise micros pour les calculs de positions car plus de précisions et donc moins d'erreurs d'intégrations
     * On utilise millis pour le dt car n'a aucun risque de saturation (micros sature 1000 fois plus vite)
     * ON S'EN BRANLE
     * */
    unsigned long m_time_millis;
    // unsigned long m_time_micros;

public:
    void setup();
    void loop();
    void reinitialise();
    Mesure_pos(Encodeur *p_encodeur_r, Encodeur *p_encodeur_l);

    /**
     * Mesure right & left encoder
     */
    long mesure_r;
    long mesure_l;

    /**
     * Position calculée avec les deux roues puis fait une moyenne
     */
    float position_theta_r = 0;
    float position_theta_l = 0;

    float position_x_r = 0;
    float position_x_l = 0;

    float position_y_r = 0;
    float position_y_l = 0;

    /**
     * Position moyenne dans le plan x, y, et theta (angle de rotation), mis à jour à chaque boucle
     */
    float position_x;
    float position_y;
    float position_theta;

    /**
     * Vitesse selon x , y et theta
     */
    float vitesse_x;
    float vitesse_y;
    float vitesse_theta;

    /**
     * vitesse des roues droites et gauche en mm/s
     */
    float vitesse_r;
    float vitesse_l;
};

#endif