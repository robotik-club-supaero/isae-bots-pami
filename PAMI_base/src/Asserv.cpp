#include <Asserv.h>
#include <Moteur.h>
#include <Arduino.h>
#include <define.h>
#include <Mesure_pos.h>

/**
 * @file Asserv.cpp
 * @brief Asservissement en vitesse et en angle du robot
 *
 */
Asserv::Asserv(Moteur *p_moteur_r, Moteur *p_moteur_l, Mesure_pos *p_mesure_pos) : m_asservPID_r(1, 0.1, 0, 255, 5), // TODO : regler l'asservissement
                                                                                   m_asservPID_l(1, 0.1, 0, 255, 5),
                                                                                   m_asservPID_angle(1, 0, 0, 255, 5)
{
    m_p_mesure_pos = p_mesure_pos;
    m_p_moteur_l = p_moteur_l;
    m_p_moteur_r = p_moteur_r;
}

void Asserv::asservissement(float vitesse_l_consigne, float vitesse_r_consigne)
{
    float erreur_l = vitesse_l_consigne - m_p_mesure_pos->vitesse_l;
    float erreur_r = vitesse_r_consigne - m_p_mesure_pos->vitesse_r;
    float output_l = m_asservPID_l.computeOutput(erreur_l, micros());
    float output_r = m_asservPID_r.computeOutput(erreur_r, micros());

    float speed_l = output_l * Kmot_l;
    float speed_r = output_r * Kmot_r;

    m_p_moteur_l->set_speed(speed_l);
    m_p_moteur_r->set_speed(speed_r);
}

void Asserv::asserv_angle(float theta_consigne)
{
    float erreur = theta_consigne - m_p_mesure_pos->position_theta;
    erreur = fmod(erreur, 2 * PI);
    if (erreur > PI)
    {
        erreur -= 2 * PI;
    }
    else if (erreur < -PI)
    {
        erreur += 2 * PI;
    }
    float output = m_asservPID_angle.computeOutput(erreur, micros());
    float speed_l = -output * Kmot_angle;
    float speed_r = output * Kmot_angle;

    m_p_moteur_l->set_speed(speed_l);
    m_p_moteur_r->set_speed(speed_r);
}

void Asserv::asserv_global(float vitesse_l_consigne, float vitesse_r_consigne, float theta_consigne)
{
    float erreur_l = vitesse_l_consigne - m_p_mesure_pos->vitesse_l;
    float erreur_r = vitesse_r_consigne - m_p_mesure_pos->vitesse_r;
    float output_l = m_asservPID_l.computeOutput(erreur_l, micros());
    float output_r = m_asservPID_r.computeOutput(erreur_r, micros());
    // Serial.println("erreur_l" + String(erreur_l));
    // Serial.println("erreur_r" + String(erreur_r));
    // Serial.println("output_l" + String(output_l));
    // Serial.println("output_r" + String(output_r));

    float erreur_theta = theta_consigne - m_p_mesure_pos->position_theta;

    // On prend l'angle modulo 2pi et on choisit le sens optimale après
    erreur_theta = fmod(erreur_theta, 2 * PI);
    if (erreur_theta > PI)
    {
        erreur_theta -= 2 * PI;
    }
    else if (erreur_theta < -PI)
    {
        erreur_theta += 2 * PI;
    }

    float output_theta = m_asservPID_angle.computeOutput(erreur_theta, micros());
    float speed_l = output_theta * Kmot_angle + output_l * Kmot_l;
    float speed_r = output_r * Kmot_r - output_theta * Kmot_angle;

    m_p_moteur_l->set_speed(speed_l);
    m_p_moteur_r->set_speed(speed_r);
}

void Asserv::setup()
{
    m_time = micros();
    m_asservPID_l.RAZ(micros());
    m_asservPID_r.RAZ(micros());
    m_asservPID_angle.RAZ(micros());
}

void Asserv::loop()
{
}