#include "esp32-hal.h"
#include <Moteur.h>

Moteur::Moteur(int EN, int IN1, int IN2, bool inv)
{
    m_EN = EN;   //pin pwm pour controler la vitesse
    m_IN1 = IN1; // deux bits pour dire au pont en H si on veut faire tourner le moteur vers l'avant ou vers l'arrière
    m_IN2 = IN2; // 01 veut dire un sens, 10 l'autre sens, 00 c'est l'arrêt
    m_inv = inv; //permet de dire si le moteur est retourné, l'avant et l'arrière seront donc inversés.
}

void Moteur::setup()
{
    // Setup les broches en output
    pinMode(m_EN, OUTPUT);  // PWM (vitesse)
    pinMode(m_IN1, OUTPUT); // Direction
    pinMode(m_IN2, OUTPUT); // Direction

    // Ecrit sur les broche l'etat initiale des PIN de directions
    digitalWrite(m_IN1, 0);
    digitalWrite(m_IN2, 1);
}

void Moteur::set_speed(int vitesse)
{

    int vitesse_a_imposer = (m_inv == true) ? -vitesse : vitesse;
    //vitesse_a_imposer > 0 => on veut avancer pour la roue qui est dans le bon sens, < 0  => on veut reculer

    if (vitesse_a_imposer == 0) // arrêt
    {
        digitalWrite(m_IN1, 0);
        digitalWrite(m_IN2, 0);
    }
    else if (vitesse_a_imposer < 0) // on veut reculer
    {
        digitalWrite(m_IN1, 1); // set le sens de rotation
        digitalWrite(m_IN2, 0);
    }
    else // on veut avancer
    {
        digitalWrite(m_IN1, 0); // set le sens de rotation
        digitalWrite(m_IN2, 1);
    }
    if (vitesse_a_imposer > 255)
    {
        analogWrite(m_EN, 255);
    }
    analogWrite(m_EN, abs(vitesse_a_imposer)); // envoie la commande de vitesse
    
}
