/**
 * @file Mesure_pos.h
 * @brief Mesure de la position et de la vitesse du robot voir readme pour plus de détail
 */

#ifndef MESURE_POS_H
#define MESURE_POS_H
#include <Arduino.h>
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
    float K_angle = 0.15; // TODO : regler les facteurs si on change la meca
    float K_r = 0.0109;   // TODO : regler les facteurs si on change la meca
    float K_l = 0.011;    // TODO : regler les facteurs si on change la meca
    /**s
     * temps entre deux mesures
     */
    long dt;

    /**
     * temps mis a jour à chaque boucle
     * 2 variables permet d'éviter les bruits/erreurs lors de la conversion millis & micros
     * On utilise micros pour les calculs de positions car plus de précisions et donc moins d'erreurs d'intégrations
     * On utilise millis pour le dt car n'a aucun risque de saturation (micros sature 1000 fois plus vite)
     * */
    unsigned long m_time_millis;
    unsigned long m_time_micros;

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
     * Position dans le plan x, y, et theta (angle de rotation), mis à jour à chaque boucle
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
     * vitesse des roues droites et gauche
     */
    float vitesse_r;
    float vitesse_l;
};

#endif