#include <Mesure_pos.h>
#include <define.h>

Mesure_pos::Mesure_pos(Encodeur *p_encodeur_r, Encodeur *p_encodeur_l)
{
    encoder_L = p_encodeur_l;
    encoder_R = p_encodeur_r;
}

void Mesure_pos::setup()
{
    dt = 10;

    encoder_L->setup();
    encoder_R->setup();
    Serial.println("Setup Done : Encodeurs");

    theta_mesuree = 0;
    x_mesuree = 0;
    y_mesuree = 0;

    mesure_encoder_l = encoder_L->mesure();
    mesure_encoder_r = encoder_R->mesure();

    m_time_millis = millis();
    m_time_micros = micros();
}

void Mesure_pos::reinitialise()
{
    encoder_L->clear_count();
    encoder_R->clear_count();

    theta_mesuree = 0;
    x_mesuree = 0;
    y_mesuree = 0;

    mesure_encoder_l = 0;
    mesure_encoder_r = 0;

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

        long current_ticks_l = encoder_L->mesure();
        long current_ticks_r = encoder_R->mesure();

        // Serial.print("Ticks L : " + String(current_ticks_l) + " | Ticks R : " + String(current_ticks_r) + " | dt (ms) : " + String(real_dt_micros / 1000.0) + " ms\n");
        // Serial.print("Mesure L : " + String(mesure_l) + " | Mesure R : " + String(mesure_r) + "\n");

        float position_l = current_ticks_l - mesure_encoder_l;
        float position_r = current_ticks_r - mesure_encoder_r;

        // formules à la zeub : TODO implémenter un vrai calcul à partir de cette formule :
        // https://math.stackexchange.com/questions/3962859/calculate-path-of-vehicle-with-two-wheels-parallel-to-each-other

        // Pour l'instant on va juste faire que des rotation roue et anti rotation d'une autre roue pour ne pas faire bouger le centre de gravité
        // Mettre K_l = K_r pour neutraliser les formules nulles et
        theta_mesuree += (position_r * K_r - position_l * K_l) * K_angle;
        x_mesuree += ((position_l * K_l + position_r * K_r) / 2.0) * cos(theta_mesuree);
        y_mesuree += ((position_l * K_l + position_r * K_r) / 2.0) * sin(theta_mesuree);

        vitesse_x = (((position_l * K_l + position_r * K_r) / 2.0) * cos(theta_mesuree) / real_dt_micros) * 1e6;
        vitesse_y = (((position_l * K_l + position_r * K_r) / 2.0) * sin(theta_mesuree) / real_dt_micros) * 1e6;

        vitesse_theta = ((position_r * K_r - position_l * K_l) * K_angle / real_dt_micros) * 1e6;
        vitesse_l = (position_l / real_dt_micros * 1e6) * K_l;
        vitesse_r = (position_r / real_dt_micros * 1e6) * K_r;

        mesure_encoder_l = current_ticks_l;
        mesure_encoder_r = current_ticks_r;

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