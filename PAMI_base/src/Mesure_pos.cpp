#include <Mesure_pos.h>
#include <define.h>

Mesure_pos::Mesure_pos(Encodeur *p_encodeur_r, Encodeur *p_encodeur_l)
{
    m_p_encoder_L = p_encodeur_l;
    m_p_encoder_R = p_encodeur_r;
}

void Mesure_pos::setup()
{
    dt = 10;

    m_p_encoder_L->setup();
    m_p_encoder_R->setup();
    Serial.println("Setup Done : Encodeurs");

    position_theta = 0;
    position_x = 0;
    position_y = 0;

    mesure_l = m_p_encoder_L->mesure();
    mesure_r = m_p_encoder_R->mesure();

    // Evite
    m_time_millis = millis();
    m_time_micros = micros();
}

void Mesure_pos::reinitialise()
{
    position_theta = 0;
    position_x = 0;
    position_y = 0;
}

void Mesure_pos::loop()
{
    if (millis() - m_time_millis >= dt)
    {
        unsigned long current_micros = micros();
        unsigned long real_dt_micros = current_micros - m_time_micros;

        m_time_micros = current_micros;
        m_time_millis = millis();

        float position_l = 0;
        float position_r = 0;

        position_l = m_p_encoder_L->mesure() - mesure_l;
        position_r = m_p_encoder_R->mesure() - mesure_r;

        // Positions
        position_theta += (position_l * K_l - position_r * K_r) * K_angle;
        position_x += ((position_l * K_l + position_r * K_r) / 2) * cos(position_theta);
        position_y += ((position_l * K_l + position_r * K_r) / 2) * sin(position_theta);

        // Vitesse
        vitesse_x = (((position_l * K_l + position_r * K_r) / 2) * cos(position_theta) / real_dt_micros) * 1e6; // En cm par seconde
        vitesse_y = (((position_l * K_l + position_r * K_r) / 2) * sin(position_theta) / real_dt_micros) * 1e6; // En cm par seconde
        vitesse_theta = ((position_l - position_r) * K_angle / real_dt_micros) * 1e6;                           // En cm radian par seconde

        // Vitesse roue droite et gauche (en cm/s)
        vitesse_l = (position_l / real_dt_micros * 1e6) * K_l;
        vitesse_r = (position_r / real_dt_micros * 1e6) * K_r;

        /**
         * Affichage des valeurs , a decommenter si on veut debug ,
         * NE PAS OUBLIER DE COMMENTER DANS LE CODE FINAL SINON LE TERMINAL SERIE INTERFERE AVEC L'ASSERVISSEMENT ET CA FAIT NIMP
         */

        // Serial.println("Position_x = " + String(position_x));
        // Serial.println("Position_y = " + String(position_y));
        // Serial.println("Position_theta = " + String(position_theta));

        // Serial.println("vitesse_x = " + String(vitesse_x));
        // Serial.println("vitesse_y = " + String(vitesse_y));
        // Serial.println("Vr=" + String(vitesse_r));
        // Serial.println("Vl=" + String(vitesse_l));

        // Serial.println("Mesure_r = " + String(mesure_r));
        // Serial.println("Mesure_l= " + String(mesure_l));

        // Maj des mesures et temps ;
        mesure_l = m_p_encoder_L->mesure();
        mesure_r = m_p_encoder_R->mesure();
    }
}