#include <Pami.h>
#include <cmath>

Pami::Pami(Moteur *p_moteur_r, Moteur *p_moteur_l, Encodeur *p_encodeur_r, Encodeur *p_encodeur_l, Mesure_pos *p_mesure_pos, Serv *p_servo, Asserv *p_asserv, Irsensor *p_ir_sensor, Ultrason *p_ultrason)
{
    m_p_moteur_r = p_moteur_r;
    m_p_moteur_l = p_moteur_l;
    m_p_encodeur_r = p_encodeur_r;
    m_p_encodeur_l = p_encodeur_l;
    m_p_mesure_pos = p_mesure_pos;
    m_p_servo = p_servo;
    m_p_asserv = p_asserv;

    // Optionnels
    m_p_ultrason = p_ultrason;
    m_p_ir_sensor = p_ir_sensor;
}

float m_time_match = millis();

/*
Fonction de diagnostic général du robot
Modes :
1 = Interrupteurs & Tirette
2 = Capteur IR (ToF)
3 = Servomoteur
4 =
5 = Encodeurs & Odométrie (À pousser à la main)
6 = Moteurs Individuels (Puissance brute)
7 = Homologation : Avance et s'arrête en fonction du capteur IR
8 = Avancer, reculer, tourner (Sans asservissement, juste pour voir si les fonctions de base marchent &  régler les gains)
*/
void Pami::test(int mode)
{
    Serial.print("\n========== LANCEMENT DU TEST MODE : ");
    Serial.print(mode);
    Serial.println(" ==========");

    switch (mode)
    {
    case 1: // --- TEST 1 : TIRETTE & INTERRUPTEURS ---
    {
        Serial.println("Test Interrupteurs... Modifiez leurs etats ! (Boucle infinie)");
        while (true)
        {
            this->print_infos_interrupteur();
            Serial.println("-------------------------");
            delay(1000); // On attend 1s pour ne pas spammer le terminal
        }
        break;
    }
    case 2: // --- TEST 2 : CAPTEUR IR ---
    {
        Serial.println("Test Capteur IR... Passez votre main devant ! (Boucle infinie)");
        while (true)
        {
            float dist = this->get_IR_distance();
            Serial.print("Distance mesuree : ");
            Serial.print(dist / 10.0);
            Serial.println(" cm");
            delay(200);
        }
        break;
    }
    case 3: // --- TEST 3 : SERVOMOTEUR ---
    {
        Serial.println("Test Servomoteur : Va-et-vient de 3 secondes");
        while (true)
        {
            // Utilise les constantes ANGLE1 et ANGLE2 de ton define.h
            this->blink_servo(1000, ANGLE1, ANGLE2);
        }
        Serial.println("Fin du test Servomoteur.");
        break;
    }
    case 5: // --- TEST 5 : ODOMETRIE ---
    {
        Serial.println("Test Encodeurs... Poussez le robot a la main ! (Boucle infinie)");
        m_p_mesure_pos->reinitialise();
        while (true)
        {
            m_p_mesure_pos->loop(); // Met a jour les calculs
            this->print_encodeur();
            this->print_position();
            this->print_speed();
            Serial.println("-------------------------");
            delay(250);
        }
        break;
    }
    case 6: // Essais roue droite & gauche indépendament
    {
        Serial.println("Test Moteurs Individuels : Attention, le robot va tester chaque roue indépendamment !");

        while (true)
        {
            Serial.println("\n-> Test Roue Droite (Vitesse 200)");
            m_p_moteur_r->set_speed(SPEED);
            m_p_moteur_l->stop();
            this->print_speed();
            this->print_encodeur();
            delay(1500);

            Serial.println("\n-> Arret");
            m_p_moteur_r->stop();
            delay(1500);

            Serial.println("\n-> Test Roue Gauche (Vitesse 200)");
            m_p_moteur_r->stop();
            m_p_moteur_l->set_speed(SPEED);
            this->print_speed();
            this->print_encodeur();
            delay(1500);

            Serial.println("\n-> Arret Definitif");
            m_p_moteur_r->stop();
            m_p_moteur_l->stop();
            delay(1500);
            Serial.println("Fin du test Moteurs Individuels.");
        }
        break;
    }
    case 7: // Homologation
    {
        Serial.println("Homologation : Le robot avance et s'arrête lorsque le capteur IR détecte un obstacle à moins de 5 cm (Boucle infinie)");

        while (true)
        {
            float dist = this->get_IR_distance();
            Serial.print("Distance mesuree : ");
            Serial.print(dist / 10.0);
            Serial.println(" cm");

            if (dist < DISTANCE_MIN && dist > 0.5) // Si un obstacle est détecté à moins de 20 cm
            {
                Serial.println("Obstacle détecté ! Arrêt du robot.");
                this->stop();
            }
            else
            {
                this->set_speed(SPEED);
            }
            delay(200);
        }
        break;
    }
    case 8:
    {
        Serial.println("Test Avancer/Reculer/Tourner... Attention, le robot va avancer, reculer puis tourner !");

        this->avancer(100);
        delay(2000);
        this->tourner(180);
    }

    default:
    {
        Serial.println("Erreur : Mode de test inconnu ! (Choisissez entre 1 et 6)");
        break;
    }
    }
}

/*
Mise à jour des positions initiales et finales de la pami en fonction de l'équipe et de sa position
*/
void Pami::config_start_position()
{
    float start_pos_x;
    float start_pos_y;

    // Equipe de la PAMI
    int read_equipe = digitalRead(PIN_READEQUIPE);
    // Position initiale de la PAMI avec deux interrupteurs;
    int int_pami_1 = digitalRead(PIN_INT_PAMI_1);
    int int_pami_2 = digitalRead(PIN_INT_PAMI_2);
    int read_num_pami = (int_pami_1 * 2) + int_pami_2 + 1;

    if (read_num_pami != num_pami || read_equipe != equipe)
    {
        String color_equipe = (read_equipe == 1) ? "JAUNE" : "BLEUE";
        // Serial.print("Nouvelle Equipe : ");
        // Serial.println(color_equipe);

        // Serial.print("PAMI n°");
        // Serial.println(num_pami);

        num_pami = read_num_pami; // On met à jour le numéro de la pami

        if (num_pami == 1)
        {
            if (read_equipe != 0)
            {
                equipe = read_equipe; // On met à jour l'équipe
                pos_init_x = J_POSITION_1_DEPART_X;
                pos_init_y = J_POSITION_1_DEPART_Y;
                pos_target_x = J_POSITION_1_FINAL_X;
                pos_target_y = J_POSITION_1_FINAL_Y;
            }
            else
            {
                pos_init_x = B_POSITION_1_DEPART_X;
                pos_init_y = B_POSITION_1_DEPART_Y;
                pos_target_x = B_POSITION_1_FINAL_X;
                pos_target_y = B_POSITION_1_FINAL_Y;
            }
        }
        else if (num_pami == 2)
        {
            if (read_equipe != 0)
            {
                pos_init_x = J_POSITION_2_DEPART_X;
                pos_init_y = J_POSITION_2_DEPART_Y;
                pos_target_x = J_POSITION_2_FINAL_X;
                pos_target_y = J_POSITION_2_FINAL_Y;
            }
            else
            {
                pos_init_x = B_POSITION_2_DEPART_X;
                pos_init_y = B_POSITION_2_DEPART_Y;
                pos_target_x = B_POSITION_2_FINAL_X;
                pos_target_y = B_POSITION_2_FINAL_Y;
            }
        }
        else if (num_pami == 3)
        {
            if (read_equipe != 0)
            {
                pos_init_x = J_POSITION_3_DEPART_X;
                pos_init_y = J_POSITION_3_DEPART_Y;
                pos_target_x = J_POSITION_3_FINAL_X;
                pos_target_y = J_POSITION_3_FINAL_Y;
            }
            else
            {
                pos_init_x = B_POSITION_3_DEPART_X;
                pos_init_y = B_POSITION_3_DEPART_Y;
                pos_target_x = B_POSITION_3_FINAL_X;
                pos_target_y = B_POSITION_3_FINAL_Y;
            }
        }
        else if (num_pami == 4)
        {
            if (read_equipe != 0)
            {
                pos_init_x = J_POSITION_4_DEPART_X;
                pos_init_y = J_POSITION_4_DEPART_Y;
                pos_target_x = J_POSITION_4_FINAL_X;
                pos_target_y = J_POSITION_4_FINAL_Y;
            }
            else
            {
                pos_init_x = B_POSITION_4_DEPART_X;
                pos_init_y = B_POSITION_4_DEPART_Y;
                pos_target_x = B_POSITION_4_FINAL_X;
                pos_target_y = B_POSITION_4_FINAL_Y;
            }
        }
    }
}

/*
Fonctions de déplacement basiques (sans asservissement, juste pour tester les fonctions de base et régler les gains K_NAIF et K_ANGLE_NAIF
*/
void Pami::go_to(float distance_x, float distance_y, int speed)
{
    this->avancer(distance_x, speed);
    delay(200);
    this->tourner(90, speed);
    delay(200);
    this->avancer(distance_y, speed);
    delay(200);
}

void Pami::avancer(float distance, int speed)
{
    if (distance < 0.)
    {
        speed = -speed;
    }

    float moving_time = K_NAIF * (abs(distance) / SPEED) * 1000;
    Serial.print("Temps estimé pour avancer de ");
    Serial.print(distance / 10.0);
    Serial.print(" cm à la vitesse de ");
    Serial.print(speed);
    Serial.print(" : ");
    Serial.print(moving_time);
    Serial.println(" ms");

    unsigned long function_start_time = millis();
    while (millis() - function_start_time < moving_time)
    {
        float dist = this->get_IR_distance();

        if (dist < DISTANCE_MIN && dist > 0.5) // Si un obstacle est détecté à moins de 20 cm
        {
            Serial.println("Obstacle détecté ! Arrêt du robot.");
            this->stop();
        }
        else
        {
            this->set_speed(speed);
        }
        delay(100);
    }
    this->stop();
}

void Pami::tourner(float angle_degres, float speed)
{
    if (angle_degres < 0)
    {
        speed = -speed;
    }

    m_p_moteur_r->set_speed(speed);
    m_p_moteur_l->set_speed(-speed);
    delay(K_ANGLE_NAIF * (abs(angle_degres) / 360.0) * 1000);
    Serial.print("Temps estimé pour tourner de ");
    Serial.print(angle_degres);
    Serial.print(" ° à la vitesse de ");
    Serial.print(speed);
    Serial.print(" : ");
    Serial.print(K_ANGLE_NAIF * (abs(angle_degres) / 360.0) * 1000);
    Serial.println(" ms");

    this->stop();
}

/*
Fonction de test pour aller a une position (x, y) [mm & mm] du plateau
*/
void Pami::go_to_asserv(float pos_final_x, float pos_final_y, int speed)
{
    m_p_mesure_pos->loop();
    pos_x = m_p_mesure_pos->position_x + pos_init_x;
    pos_y = m_p_mesure_pos->position_y + pos_init_y;
    distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));

    while (distance_target > EPSP)
    {
        Serial.print("Distance : ");
        Serial.print(distance_target / 10.0);
        Serial.println(" cm");

        if (millis() - m_time_match >= ENDTIME)
        {
            this->stop();
            return;
        }

        m_p_mesure_pos->loop();
        pos_x = m_p_mesure_pos->position_x + pos_init_x;
        pos_y = m_p_mesure_pos->position_y + pos_init_y;

        distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));
        angle = atan2(pos_final_y - pos_y, pos_final_x - pos_x);

        float erreur_angle = angle - m_p_mesure_pos->position_theta;
        erreur_angle = fmod(erreur_angle, 2 * PI);
        if (erreur_angle > PI)
            erreur_angle -= 2 * PI;
        else if (erreur_angle < -PI)
            erreur_angle += 2 * PI;

        float vitesse_avance = speed;

        if (distance_target < 50.0)
        {
            vitesse_avance = speed / 2.0;
        }

        m_p_asserv->asserv_global(vitesse_avance, vitesse_avance, angle);

        delay(10);
    }

    // Arrêt total une fois la cible atteinte
    this->stop();
}

void Pami::stop()
{
    m_p_moteur_r->stop();
    m_p_moteur_l->stop();
}

/*
Fonction de test pour avancer d'une certaine distance
*/
void Pami::avancer_asserv(float distance, int speed)
{
    m_p_mesure_pos->loop();
    float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
    float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
    float start_angle = m_p_mesure_pos->position_theta;

    float distance_traveled = 0.0;

    while (distance_traveled < distance)
    {
        m_p_mesure_pos->loop();
        float pos_x = m_p_mesure_pos->position_x + pos_init_x;
        float pos_y = m_p_mesure_pos->position_y + pos_init_y;

        float dx = pos_x - start_pos_x;
        float dy = pos_y - start_pos_y;

        // 2. On projette ce déplacement sur l'axe du robot (produit scalaire)
        distance_traveled = abs(dx * cos(start_angle) + dy * sin(start_angle));

        Serial.print("distance parcourue : ");
        Serial.println(distance_traveled);

        Serial.println("dx : " + String(dx) + " | dy : " + String(dy));

        m_p_asserv->asserv_global(speed, speed, start_angle);
        delay(10);
    }

    // On s'arrête quand on est arrivés
    this->stop();
}

/*
Fonction de test pour reculer d'une distance en x et en y
*/
void Pami::reculer_asserv(float distance, int speed)
{
    m_p_mesure_pos->loop();
    float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
    float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
    float start_angle = m_p_mesure_pos->position_theta;

    float distance_traveled = 0.0;

    while (distance_traveled < distance)
    {
        m_p_mesure_pos->loop();
        float pos_x = m_p_mesure_pos->position_x + pos_init_x;
        float pos_y = m_p_mesure_pos->position_y + pos_init_y;

        float dx = pos_x - start_pos_x;
        float dy = pos_y - start_pos_y;

        // 2. On projette ce déplacement sur l'axe du robot (produit scalaire)
        // Comme le robot recule, cette valeur va devenir de plus en plus NÉGATIVE
        distance_traveled = abs(dx * cos(start_angle) + dy * sin(start_angle));

        Serial.print("Distance parcourue (à l'envers) : ");
        Serial.println(distance_traveled);

        m_p_asserv->asserv_global(-speed, -speed, start_angle);
        delay(10);
    }

    // On s'arrête quand on est arrivés
    this->stop();
}

/*
Fonction de test qui fait tourner la pami de [theta_target] degrés
*/
void Pami::tourner_asserv(float angle_degres, float speed)
{
    m_p_mesure_pos->loop();

    float angle_rad = angle_degres * (PI / 180.0);
    float target_theta = m_p_mesure_pos->position_theta + angle_rad;
    float current_theta = m_p_mesure_pos->position_theta;
    float erreur_angle = fmod(target_theta - current_theta, 2 * PI);
    if (erreur_angle > PI)
    {
        erreur_angle -= 2 * PI;
    }
    else if (erreur_angle < -PI)
    {
        erreur_angle += 2 * PI;
    }

    while (abs(erreur_angle) > EPSA)
    {
        m_p_mesure_pos->loop();
        current_theta = abs(m_p_mesure_pos->position_theta);
        erreur_angle = fmod(target_theta - current_theta, 2 * PI);

        Serial.print("Current angle : ");
        Serial.println(current_theta * (180.0 / PI));
        Serial.print("Erreur angle : ");
        Serial.println(erreur_angle * (180.0 / PI));

        m_p_asserv->asserv_angle(target_theta);
        delay(10);
    }

    this->stop();
}

/*
Fonction pour aller a une position (x, y) du plateau
*/
bool Pami::go_to_with_obstacle(float pos_final_x, float pos_final_y, int speed)
{
    m_p_mesure_pos->loop();
    pos_x = m_p_mesure_pos->position_x + pos_init_x;
    pos_y = m_p_mesure_pos->position_y + pos_init_y;
    distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));

    while (distance_target > EPSP)
    {
        if (millis() - m_time_match >= ENDTIME)
        {
            this->stop();
            return false;
        }

        if (m_p_ir_sensor != nullptr)
        {
            float dist_obstacle = this->get_IR_distance();

            if (dist_obstacle > 10.0 && dist_obstacle < DISTANCE_MIN)
            {
                this->stop();
                delay(10);
                continue;
            }
        }

        m_p_mesure_pos->loop();
        pos_x = m_p_mesure_pos->position_x + pos_init_x;
        pos_y = m_p_mesure_pos->position_y + pos_init_y;

        distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));
        angle = atan2(pos_final_y - pos_y, pos_final_x - pos_x);

        float erreur_angle = angle - m_p_mesure_pos->position_theta;
        erreur_angle = fmod(erreur_angle, 2 * PI);
        if (erreur_angle > PI)
            erreur_angle -= 2 * PI;
        else if (erreur_angle < -PI)
            erreur_angle += 2 * PI;

        float vitesse_avance = speed;

        if (distance_target < 50.0)
        {
            vitesse_avance = speed / 2.0;
        }

        if (abs(erreur_angle) > 0.45)
        {
            vitesse_avance = 0;
        }

        m_p_asserv->asserv_global(vitesse_avance, vitesse_avance, angle);

        delay(10);
    }

    this->stop();
    return true;
}

/*
Fonction pour avancer d'une certaine distance
*/
bool Pami::avancer_with_obstacle(float distance, int speed)
{
    m_p_mesure_pos->loop();
    float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
    float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
    float start_angle = m_p_mesure_pos->position_theta;

    float distance_parcourue = 0.0;

    while (distance_parcourue < distance)
    {
        // 1. Sécurité temps de match
        if (millis() - m_time_match >= ENDTIME)
        {
            m_p_asserv->asserv_global(0, 0, start_angle);
            return false;
        }

        // 2. Distance au prochain obstacle
        if (m_p_ir_sensor != nullptr)
        {
            float dist_obstacle = this->get_IR_distance();
            Serial.print("Distance obstacle : ");
            Serial.print(dist_obstacle / 10.0);

            if (dist_obstacle > 0.01 && dist_obstacle < DISTANCE_MIN)
            {
                this->stop();
                // m_p_asserv->asserv_global(0, 0, start_angle);
                Serial.println("Arrêt du robot.");
                delay(10);
                continue; // Repart au début du "do" sans avancer
            }
        }

        // 3. Position actuelle
        m_p_mesure_pos->loop();
        float pos_x = m_p_mesure_pos->position_x + pos_init_x;
        float pos_y = m_p_mesure_pos->position_y + pos_init_y;

        // 4. Déplacement
        float dx = pos_x - start_pos_x;
        float dy = pos_y - start_pos_y;

        distance_parcourue = dx * cos(start_angle) + dy * sin(start_angle);

        Serial.print("Distance parcourue : ");
        Serial.println(distance_parcourue);

        m_p_asserv->asserv_global(speed, speed, start_angle);
        delay(10);
    }

    // On s'arrête quand on est arrivés
    this->stop();
    return true;
}

/*
Allume les deux moteurs à une vitesse en (entre 0 et 255)
*/
void Pami::set_speed(float speed)
{
    // Si on règle les gains askip c'est mieux
    // m_p_asserv->asservissement(speed, speed);
    m_p_moteur_r->set_speed(speed);
    m_p_moteur_l->set_speed(speed);
}

/*
Fonction pour bouger le servo entre deux angles en un temps donné
*/
void Pami::blink_servo(long temps_blink, int angle1, int angle2)
{
    m_p_servo->blink(temps_blink, angle1, angle2);
}

/*
Fonction qui affiche la distance au prochain obstacle détectée par le capteur ultrason
*/
double Pami::get_ultrason_distance()
{
    if (m_p_ultrason == nullptr)
    {
        Serial.println("Pas de capteur ultrason");
        return -1;
    }
    else
    {
        return m_p_ultrason->m_distance;
    }
}

/*
En mm
Fonction qui retourne la distance minimal au prochain obstacle détectée par le capteur infrarouge (ToF)
*/
double Pami::get_IR_distance()
{
    if (DISABLE_OBS)
    {
        return -1;
    }

    if (m_p_ir_sensor == nullptr)
    {
        Serial.println("Pas de capteur infrarouge");
        return -1;
    }
    else
    {
        m_p_ir_sensor->loop();
        return m_p_ir_sensor->ir_minimum_distance;
    }
}

void Pami::print_position()
{
    if (millis() - m_time_log > 250)
    {
        m_p_mesure_pos->loop();
        Serial.print("Pos X : " + String(m_p_mesure_pos->position_x / 10) + " cm");
        Serial.print(" | Pos Y : " + String(m_p_mesure_pos->position_y / 10) + " cm");
        Serial.println(" | Theta : " + String(m_p_mesure_pos->position_theta * (180.0 / PI)) + "°");
    }
}

void Pami::print_encodeur()
{
    if (millis() - m_time_log > 250)
    {
        Serial.print("Encodeur gauche : " + String(m_p_encodeur_l->mesure()));
        Serial.println(" | Encodeur droit : " + String(m_p_encodeur_r->mesure()));
    }
}

void Pami::print_speed()
{
    if (millis() - m_time_log > 275)
    {
        m_p_mesure_pos->loop();
        Serial.print("Vitesse droite : " + String(m_p_mesure_pos->vitesse_r / 10) + " cm/s | Vitesse gauche : " + String(m_p_mesure_pos->vitesse_l / 10) + " cm/s");
        Serial.print(" | Vitesse en x : " + String(m_p_mesure_pos->vitesse_x / 10) + " cm/s | Vitesse en y : " + String(m_p_mesure_pos->vitesse_y / 10) + " cm/s");
        Serial.println(" | Vitesse angulaire : " + String(m_p_mesure_pos->vitesse_theta) + " rad/s");
    }
}

void Pami::print_log()
{
    if (m_time_log + 500 < millis()) // Log toutes les secondes
    {
        Serial.println("Distance Ir: " + String(this->get_IR_distance()) + " mm");
        this->print_speed();
        this->print_position();
        this->print_encodeur();
        this->print_infos_interrupteur();

        m_time_log = millis();
    }
}

void Pami::print_infos_interrupteur()
{
    equipe = digitalRead(PIN_READEQUIPE);
    tirette = digitalRead(PIN_TIRETTE);
    int_pami_1 = digitalRead(PIN_INT_PAMI_1);
    int_pami_2 = digitalRead(PIN_INT_PAMI_2);

    if (tirette == 1)
    {
        Serial.println("Tirette : Mise en place");
    }
    else
    {
        Serial.println("Tirette : Enlevée");
    }

    if (equipe == 1)
    {
        Serial.println("Equipe : JAUNE");
    }
    else
    {
        Serial.println("Equipe : BLEUE");
    }

    if (int_pami_1 == 0 && int_pami_2 == 0)
    {
        Serial.println("PAMI n°1 - collé au mur");
    }
    else if (int_pami_1 == 0 && int_pami_2 == 1)
    {
        Serial.println("PAMI n°2");
    }
    else if (int_pami_1 == 1 && int_pami_2 == 0)
    {
        Serial.println("PAMI n°3");
    }
    else if (int_pami_1 == 1 && int_pami_2 == 1)
    {
        Serial.println("PAMI n°4 - plus éloigné du mur");
    }
}

std::tuple<float, float, unsigned long>
Pami::avancer_asservi(float tick_distance, float old_ticks_l, float old_ticks_r, unsigned long oldtime)
{
    /* But du gain proportionnel : faire une correction proportionnelle à l'erreur.
    En gros :
    erreur = ticksG - ticksD
    correction = Kp * erreur

    Puis on ajuste le pwm :
    pwmG = pwmBase - correction
    pwmD = pwmBase + correction
    */

    // Si l’intervalle n’est pas écoulé → on ne fait rien
    if ((millis() - oldtime) < interval_asserv)
    {
        return std::make_tuple(old_ticks_l, old_ticks_r, oldtime);
    }

    // Sinon, c'est qu'on vient de dépasser l'invervalle d'asservissement.
    // --- Mesures actuelles ---
    float ticks_l = m_p_encodeur_l->mesure();
    float ticks_r = m_p_encodeur_r->mesure();

    Serial.print("\n\nTicks L : ");
    Serial.print(ticks_l);
    Serial.print(" | Ticks R : ");
    Serial.println(ticks_r);

    // --- Consignes (position) ---
    float consigne_l = tick_distance + old_ticks_l;
    float consigne_r = old_ticks_r + tick_distance;

    Serial.print("Consigne L : ");
    Serial.print(consigne_l);
    Serial.print(" | Consigne R : ");
    Serial.println(consigne_r);

    // --- Erreurs ---
    float erreur_l = consigne_l - ticks_l;
    float erreur_r = consigne_r - ticks_r;

    Serial.print("Erreur L : ");
    Serial.print(erreur_l);
    Serial.print(" | Erreur R : ");
    Serial.println(erreur_r);

    float erreur_l_normalisee = erreur_l / nb_ticks_par_sec_max;
    float erreur_r_normalisee = erreur_r / nb_ticks_par_sec_max;

    Serial.print("Erreur L normalisée : ");
    Serial.print(erreur_l_normalisee);
    Serial.print(" | Erreur R normalisée : ");
    Serial.println(erreur_r_normalisee);

    // l'erreur peut-être négative, et est entre 0 et 1

    // --- Correction ---
    // on multiplie l'erreur par 255 pour avoir une erreur en vitesse pwm
    // et on multiplie aussi par Kp pour avoir la correction selon le principe de base du correcteur proportionnel
    int pwmL = Kp * 255 * erreur_l_normalisee;
    int pwmR = Kp * 255 * erreur_r_normalisee;

    Serial.print("Correction L : ");
    Serial.print(pwmL);
    Serial.print(" | Correction R : ");
    Serial.println(pwmR);

    pwmL = constrain(pwmL, -255, 255); // par sécurité mais normalement ça dépasse pas
    pwmR = constrain(pwmR, -255, 255);

    // --- Commande moteurs ---
    m_p_moteur_l->set_speed(pwmL);
    m_p_moteur_r->set_speed(pwmR);

    // --- Condition d’arrêt en ticks ---
    if (abs(erreur_l) < marge_erreur_ticks && abs(erreur_r) < marge_erreur_ticks)
    {
        m_p_moteur_l->set_speed(0);
        m_p_moteur_r->set_speed(0);
    }

    // On renvoie les nouvelles valeurs
    return std::make_tuple(ticks_l, ticks_r, oldtime);
}
