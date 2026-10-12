/**
 * @file Moteur.h
 * @brief Classe pour controler un moteur
 *
 */

#include <Arduino.h>

#ifndef MOTEUR_H
#define MOTEUR_H

class Moteur
{
private:
    int m_EN;   // pin de la pwm
    int m_IN1;  // pin de direction 1
    int m_IN2;  // pin de direction 2
    bool m_inv; // flag pour l'inversion de la direction pour le moteur qui est à l'envers
    long m_vitesse; //vitesse entre -255 et 255
    
    // todo : à implémenter dans le futur
    // float K_conv; // Facteur pour passer d'un entier entre -255 ET 255 à une vitesse en cm/s 

public:

    /**
     * @brief constructeur
     */
    Moteur(int EN, int IN1, int IN2, bool inv = false);

    /**
     * @brief Initialisation du moteur
     */
    void setup();

    /**
     * @brief met la vitesse du moteur à vitesse
     *
     * @param vitesse : speed of the motor is a int between -255 and 255
     */
    void set_speed(int vitesse);
};

#endif