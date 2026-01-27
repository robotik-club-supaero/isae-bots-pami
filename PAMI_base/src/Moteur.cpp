#include <Moteur.h>

Moteur::Moteur(int EN, int IN1, int IN2)
{
    m_EN = EN;
    m_IN1 = IN1;
    m_IN2 = IN2;
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
    if (vitesse < 0)
    {
        digitalWrite(m_IN1, 1); // set le sens de rotation
        digitalWrite(m_IN2, 0);
        m_vitesse = -vitesse; // Setup la vitesse en valeur absolue
    }
    else
    {
        digitalWrite(m_IN1, 0); // set le sens de rotation
        digitalWrite(m_IN2, 1);
        m_vitesse = vitesse; // Setup la vitesse en valeur absolue
    }
    if (m_vitesse > 255)
    {
        m_vitesse = 255; // contraint la vitesse en valeur absolue
    }
    analogWrite(m_EN, m_vitesse); // envoie la command de vitesse
}

void Moteur::stop(){
    digitalWrite(m_IN1, 0);
    digitalWrite(m_IN2, 0);
    analogWrite(m_EN, 0);
}

void Moteur::loop()
{
}

