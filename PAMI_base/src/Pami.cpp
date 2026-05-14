#include <Pami.h>
#include <cmath>

Pami::Pami(int *etape_globale, Moteur *moteur_r, Moteur *moteur_l, Encodeur *encodeur_r, Encodeur *encodeur_l, Mesure_pos *mesure_pos, Serv *servo, Irsensor *ir_sensor, Ultrason *ultrason)
{
    p_moteur_r = moteur_r;
    p_moteur_l = moteur_l;
    p_encodeur_r = encodeur_r;
    p_encodeur_l = encodeur_l;
    p_mesure_pos = mesure_pos;
    p_servo = servo;
    p_etape_globale = etape_globale;

    // Optionnels
    p_ultrason = ultrason;
    p_ir_sensor = ir_sensor;
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
        p_mesure_pos->reinitialise();
        while (true)
        {
            p_mesure_pos->loop(); // Met a jour les calculs
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
            p_moteur_r->set_speed(SPEED);
            p_moteur_l->stop();
            this->print_speed();
            this->print_encodeur();
            delay(1500);

            Serial.println("\n-> Arret");
            p_moteur_r->stop();
            delay(1500);

            Serial.println("\n-> Test Roue Gauche (Vitesse 200)");
            p_moteur_r->stop();
            p_moteur_l->set_speed(SPEED);
            this->print_speed();
            this->print_encodeur();
            delay(1500);

            Serial.println("\n-> Arret Definitif");
            p_moteur_r->stop();
            p_moteur_l->stop();
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

    p_moteur_r->set_speed(speed);
    p_moteur_l->set_speed(-speed);
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

void Pami::stop()
{
    p_moteur_r->stop();
    p_moteur_l->stop();
}

/*
Allume les deux moteurs à une vitesse en (entre 0 et 255)
*/
void Pami::set_speed(float speed)
{
    // Si on règle les gains askip c'est mieux
    // p_asserv->asservissement(speed, speed);
    p_moteur_r->set_speed(speed);
    p_moteur_l->set_speed(speed);
}

/*
Fonction pour bouger le servo entre deux angles en un temps donné
*/
void Pami::blink_servo(long temps_blink, int angle1, int angle2)
{
    p_servo->blink(temps_blink, angle1, angle2);
}

/*
Fonction qui affiche la distance au prochain obstacle détectée par le capteur ultrason
*/
double Pami::get_ultrason_distance()
{
    if (p_ultrason == nullptr)
    {
        Serial.println("Pas de capteur ultrason");
        return -1;
    }
    else
    {
        return p_ultrason->m_distance;
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

    if (p_ir_sensor == nullptr)
    {
        Serial.println("Pas de capteur infrarouge");
        return -1;
    }
    else
    {
        p_ir_sensor->loop();
        return p_ir_sensor->ir_minimum_distance;
    }
}

void Pami::print_position()
{
    if (millis() - m_time_log > 250)
    {
        p_mesure_pos->loop();
        Serial.print("Pos X : " + String(p_mesure_pos->x_mesuree / 10) + " cm");
        Serial.print(" | Pos Y : " + String(p_mesure_pos->y_mesuree / 10) + " cm");
        Serial.println(" | Theta : " + String(p_mesure_pos->theta_mesuree * (180.0 / PI)) + "°");
    }
}

void Pami::print_encodeur()
{
    if (millis() - m_time_log > 250)
    {
        Serial.print("Encodeur gauche : " + String(p_encodeur_l->mesure()));
        Serial.println(" | Encodeur droit : " + String(p_encodeur_r->mesure()));
    }
}

void Pami::print_speed()
{
    if (millis() - m_time_log > 275)
    {
        p_mesure_pos->loop();
        Serial.print("Vitesse droite : " + String(p_mesure_pos->vitesse_r / 10) + " cm/s | Vitesse gauche : " + String(p_mesure_pos->vitesse_l / 10) + " cm/s");
        Serial.print(" | Vitesse en x : " + String(p_mesure_pos->vitesse_x / 10) + " cm/s | Vitesse en y : " + String(p_mesure_pos->vitesse_y / 10) + " cm/s");
        Serial.println(" | Vitesse angulaire : " + String(p_mesure_pos->vitesse_theta) + " rad/s");
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

std::tuple<float, float, unsigned long> Pami::avancer_asservi(int etape_d_appel, float consigne_cm_l, float consigne_cm_r, float old_ticks_l, float old_ticks_r, unsigned long oldtime)
{
    // Si c'est pas l'étape à laquelle on veut l'appeler,
    // aucune des variables du main n'est modifiée
    if (etape_d_appel != *p_etape_globale)
    {
        return std::make_tuple(old_ticks_l, old_ticks_r, oldtime);
    }
    /* But du gain proportionnel : faire une correction proportionnelle à l'erreur.
    En gros :s
    erreur = ticksG - ticksD
    correction = Kp * erreur

    Puis on ajuste le pwm :
    pwmG = pwmBase - correction
    pwmD = pwmBase + correction
    */

    // Si l’intervalle n’est pas écoulé -> on ne fait rien
    if (millis() - oldtime < interval_asserv)
    {
        return std::make_tuple(old_ticks_l, old_ticks_r, oldtime);
    }
    else
    {
        // Sinon, c'est qu'on vient de dépasser l'invervalle d'asservissement.
        // il faut donc asservir de nouveau

        // --- Mesures actuelles ---
        float ticks_l = p_encodeur_l->mesure();
        float ticks_r = p_encodeur_r->mesure();
        Serial.print("ticks_l : ");
        Serial.print(ticks_l);
        Serial.print("\t ticks_r : ");
        Serial.println(ticks_r);

        // --- Erreurs ---
        // float erreur_l = consigne_l - ticks_l;
        // float erreur_r = consigne_r - ticks_r;

        float erreur = ticks_l - ticks_r;

        // float erreur_l_normalisee = erreur_l/nb_ticks_par_sec_max;
        // float erreur_r_normalisee = erreur_r/nb_ticks_par_sec_max;

        // l'erreur peut-être négative, et est entre 0 et 1

        // --- Correction ---
        // on multiplie l'erreur par 255 pour avoir une erreur en vitesse pwm
        // et on multiplie aussi par Kp pour avoir la correction selon le principe de base du correcteur proportionnel
        // int pwmL = Kp * SPEED * erreur_l_normalisee;
        // int pwmR = Kp * SPEED * erreur_r_normalisee;

        int pwmR = SPEED + Kp * erreur;
        int pwmL = SPEED - Kp * erreur;

        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);

        // --- Commande moteurs ---
        p_moteur_l->set_speed(pwmL);
        p_moteur_r->set_speed(pwmR);

        // --- Condition d’arrêt en ticks ---
        if (abs(consigne_cm_l * GAIN_CM_TO_TICKS - ticks_l) < marge_erreur_ticks && abs(consigne_cm_r * GAIN_CM_TO_TICKS - ticks_r) < marge_erreur_ticks)
        {
            // ON RENTRE !!
            p_moteur_l->set_speed(0);
            p_moteur_r->set_speed(0);
            (*p_etape_globale)++;
        }

        // On renvoie les nouvelles valeurs
        return std::make_tuple(ticks_l, ticks_r, millis());
    }
}

std::tuple<float, float, unsigned long> Pami::tourner_asservi(int etape_d_appel, float consigne_angle, float old_ticks_l, float old_ticks_r, unsigned long oldtime)
{
    // Si c'est pas l'étape à laquelle on veut l'appeler,
    // aucune des variables du main n'est modifiée
    if (etape_d_appel != *p_etape_globale)
    {
        return std::make_tuple(old_ticks_l, old_ticks_r, oldtime);
    }
    /* But du gain proportionnel : faire une correction proportionnelle à l'erreur.
    En gros :s
    erreur = ticksG - ticksD
    correction = Kp * erreur

    Puis on ajuste le pwm :
    pwmG = pwmBase - correction
    pwmD = pwmBase + correction
    */

    // Si l’intervalle n’est pas écoulé -> on ne fait rien
    if (millis() - oldtime < interval_asserv)
    {
        return std::make_tuple(old_ticks_l, old_ticks_r, oldtime);
    }
    else
    {
        // Sinon, c'est qu'on vient de dépasser l'invervalle d'asservissement.
        // il faut donc asservir de nouveau

        // --- Mesures actuelles ---
        float ticks_l = p_encodeur_l->mesure();
        float ticks_r = p_encodeur_r->mesure();
        // Serial.print("ticks_l : ");
        // Serial.print(ticks_l);
        // Serial.print("\t ticks_r : ");
        // Serial.print(ticks_r);

        // --- Erreurs ---
        // float erreur_l = consigne_l - ticks_l;
        // float erreur_r = consigne_r - ticks_r;

        float erreur = ticks_l - ticks_r;

        // float erreur_l_normalisee = erreur_l/nb_ticks_par_sec_max;
        // float erreur_r_normalisee = erreur_r/nb_ticks_par_sec_max;

        // l'erreur peut-être négative, et est entre 0 et 1

        // --- Correction ---
        // on multiplie l'erreur par 255 pour avoir une erreur en vitesse pwm
        // et on multiplie aussi par Kp pour avoir la correction selon le principe de base du correcteur proportionnel
        // int pwmL = Kp * SPEED * erreur_l_normalisee;
        // int pwmR = Kp * SPEED * erreur_r_normalisee;

        int pwmR = SPEED + Kp * erreur;
        int pwmL = SPEED - Kp * erreur;

        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);

        // --- Condition d’arrêt en ticks ---
        // --- Commande moteurs ---
        if (consigne_angle > 0)
        {
            p_moteur_r->set_speed(pwmR);
            if (abs(consigne_angle * GAIN_ANGLE_TO_TICKS - ticks_l) < marge_erreur_ticks)
            {
                // ON RENTRE !!
                p_moteur_r->set_speed(0);
                (*p_etape_globale)++;
            }
        }
        else
        {
            p_moteur_l->set_speed(pwmL);
            if (abs(consigne_angle * GAIN_ANGLE_TO_TICKS - ticks_r) < marge_erreur_ticks)
            {
                // ON RENTRE !!
                p_moteur_l->set_speed(0);
                (*p_etape_globale)++;
            }
        }

        // On renvoie les nouvelles valeurs
        return std::make_tuple(ticks_l, ticks_r, millis());
    }
}