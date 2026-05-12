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

    m_time_millis = millis();
    m_time_micros = micros();
}

void Mesure_pos::reinitialise()
{
    m_p_encoder_L->clear_count();
    m_p_encoder_R->clear_count();

    position_theta = 0;
    position_x = 0;
    position_y = 0;

    mesure_l = 0;
    mesure_r = 0;

    m_time_millis = millis();
    m_time_micros = micros();

    Serial.println("Odométrie et encodeurs réinitialisés.");
}

void Mesure_pos::loop()
{
    if (millis() - m_time_millis >= dt)
    {
        unsigned long current_micros = micros();
        unsigned long real_dt_micros = current_micros - m_time_micros;

        m_time_micros = current_micros;
        m_time_millis = millis();

        long current_ticks_l = m_p_encoder_L->mesure();
        long current_ticks_r = m_p_encoder_R->mesure();

        // Serial.print("Ticks L : " + String(current_ticks_l) + " | Ticks R : " + String(current_ticks_r) + " | dt (ms) : " + String(real_dt_micros / 1000.0) + " ms\n");
        // Serial.print("Mesure L : " + String(mesure_l) + " | Mesure R : " + String(mesure_r) + "\n");

        float position_l = current_ticks_l - mesure_l;
        float position_r = current_ticks_r - mesure_r;

        position_theta += (position_r * K_r - position_l * K_l) * K_angle;
        position_x += ((position_l * K_l + position_r * K_r) / 2.0) * cos(position_theta);
        position_y += ((position_l * K_l + position_r * K_r) / 2.0) * sin(position_theta);

        vitesse_x = (((position_l * K_l + position_r * K_r) / 2.0) * cos(position_theta) / real_dt_micros) * 1e6;
        vitesse_y = (((position_l * K_l + position_r * K_r) / 2.0) * sin(position_theta) / real_dt_micros) * 1e6;

        vitesse_theta = ((position_r * K_r - position_l * K_l) * K_angle / real_dt_micros) * 1e6;
        vitesse_l = (position_l / real_dt_micros * 1e6) * K_l;
        vitesse_r = (position_r / real_dt_micros * 1e6) * K_r;

        mesure_l = current_ticks_l;
        mesure_r = current_ticks_r;

        /**
         * Affichage des valeurs , a decommenter si on veut debug ,
         * NE PAS OUBLIER DE COMMENTER DANS LE CODE FINAL SINON LE TERMINAL SERIE INTERFERE AVEC L'ASSERVISSEMENT ET CA FAIT NIMP
         */
        // Serial.println("Position_x = " + String(position_x));
        // Serial.println("Position_y = " + String(position_y));
        // Serial.println("Position_theta = " + String(position_theta * 180 / PI));

        // Serial.println("vitesse_x = " + String(vitesse_x));
        // Serial.println("vitesse_y = " + String(vitesse_y));
        // Serial.println("Vr=" + String(vitesse_r));
        // Serial.println("Vl=" + String(vitesse_l));

        // Serial.println("Mesure_r = " + String(mesure_r));
        // Serial.println("Mesure_l= " + String(mesure_l));
    }
}