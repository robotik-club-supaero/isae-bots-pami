#include <Mesure_pos.h>
#include <define.h>

float angle_diff(float angle_1, float angle_2){
    return fmod(angle_1 - angle_2 + 3.14, 2 * PI) - PI ;
}

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
    Serial.println("Encoders setup");

    position_theta = 0;
    position_x = 0;
    position_y = 0;

    if (INV_L == 1 ){
        last_mesure_l = -m_p_encoder_L->mesure();
    }
    else{
        last_mesure_l = m_p_encoder_L->mesure();
    }
    if (INV_R == 1 ){
        last_mesure_r = -m_p_encoder_R->mesure();
    }
    else{
        last_mesure_r = m_p_encoder_R->mesure();
    }

    m_time = millis();
}

void Mesure_pos::reinitialise() {
    position_theta = 0;
    position_x = 0;
    position_y = 0;
}

void Mesure_pos::loop()
{
    if (millis() - m_time >= dt)
    {
        int real_dt_micros = micros() - m_time * 1000; // ~dt // but why not dt ? -> easier ?
        float displacement_l = 0;
        float displacement_r = 0;

        // Displacement between (n + 1) * dt & n * dt 
        if (INV_L == 1 ){
            displacement_l = -m_p_encoder_L->mesure() - last_mesure_l; 
        }
        else{
            displacement_l = m_p_encoder_L->mesure() - last_mesure_l; 
        }
        if (INV_R == 1 ){
            displacement_r = -m_p_encoder_R->mesure() - last_mesure_r;
        }
        else{
            displacement_r = m_p_encoder_R->mesure() - last_mesure_r;
        }

        // Positions
        position_theta = fmod(position_theta + (displacement_l * K_l - displacement_r * K_r) * K_angle, 2 * PI);
        position_x = position_x + ((displacement_l * K_l + displacement_r * K_r) / 2) * cos(position_theta);
        position_y = position_y + ((displacement_l * K_l + displacement_r * K_r) / 2) * sin(position_theta);
        
        // Vitesse
        Vitesse_x = (((displacement_l * K_l + displacement_r * K_r) / 2) * cos(position_theta) / real_dt_micros) * 1e6; // En cm metre  par seconde
        Vitesse_y = (((displacement_l * K_l + displacement_r * K_r) / 2) * sin(position_theta) / real_dt_micros) * 1e6; // En cm metre  par seconde
        Vitesse_theta = ((displacement_l * K_l - displacement_r * K_r) * K_angle / real_dt_micros) * 1e6;                           // En cm radian par seconde

        // Vitesse roue droite et gauche (en cm/s)
        vitesse_l = (displacement_l / real_dt_micros * 1e6) * K_l;
        vitesse_r = (displacement_r / real_dt_micros * 1e6) * K_r;

        /**
         * Affichage des valeurs , a decommenter si on veut debug , 
         * NE PAS OUBLIER DE COMMENTER DANS LE CODE FINAL SINON LE TERMINAL SERIE INTERFERE AVEC L'ASSERVISSEMENT ET CA FAIT NIMP
        */
       
        // Serial.println("Position_x = " + String(position_x));
        // Serial.println("Position_y = " + String(position_y));
        // Serial.println("Position_theta = " + String(position_theta));

        //Serial.println("Vitesse_x = " + String(Vitesse_x));
        //Serial.println("Vitesse_y = " + String(Vitesse_y));
        //Serial.println("Vr=" + String(vitesse_r));
        //Serial.println("Vl=" + String(vitesse_l));

        // Serial.println("Mesure_r = " + String(mesure_r));
        // Serial.println("Mesure_l= " + String(mesure_l));
        
        // Maj des mesures et temps ;
        if (INV_L == 1 ){
            last_mesure_l = -m_p_encoder_L->mesure();
        }
        else{
            last_mesure_l = m_p_encoder_L->mesure();
        }
        if (INV_R == 1 ){
            last_mesure_r = -m_p_encoder_R->mesure();
        }
        else{
            last_mesure_r = m_p_encoder_R->mesure();
        }
        m_time = millis();
    }
}