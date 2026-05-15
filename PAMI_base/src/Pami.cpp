#include <Pami.h>
#include <cmath>

Pami::Pami(Moteur *moteur_r, Moteur *moteur_l, Encodeur *encodeur_r, Encodeur *encodeur_l, Serv *servo, Irsensor *ir_sensor, Ultrason *ultrason)
{
    p_moteur_r = moteur_r;
    p_moteur_l = moteur_l;
    p_encodeur_r = encodeur_r;
    p_encodeur_l = encodeur_l;
    p_servo = servo;
    p_ultrason = ultrason;
    p_ir_sensor = ir_sensor;
}

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
    // case 4: // --- Test 4 : GAINS ASSERVISSEMENT ---
    // {
    //     float time = millis();
    //     float oldtime = time;
    //     while (true)
    //     {
    //         time = this->avancer_asservi(0, 5000, oldtime);
    //         this->print_encodeur();
    //         oldtime = time;
    //     }
    //     break;
    // }
    case 5: // --- TEST 5 : ODOMETRIE ---
    {
        Serial.println("Test Encodeurs... Poussez le robot a la main ! (Boucle infinie)");
        while (true)
        {
            this->print_encodeur();
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
            this->print_encodeur();
            delay(1500);

            Serial.println("\n-> Arret");
            p_moteur_r->stop();
            delay(1500);

            Serial.println("\n-> Test Roue Gauche (Vitesse 200)");
            p_moteur_r->stop();
            p_moteur_l->set_speed(SPEED);
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

void Pami::update_setup()
{
    // Equipe de la PAMI
    int read_equipe = digitalRead(PIN_READEQUIPE);
    // Position initiale de la PAMI avec deux interrupteurs;
    int int_pami_1 = digitalRead(PIN_INT_PAMI_1);
    int int_pami_2 = digitalRead(PIN_INT_PAMI_2);
    int read_num_pami = (int_pami_1 * 2) + int_pami_2 + 1;
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

void Pami::print_encodeur()
{
    if (millis() - m_time_log > 250)
    {
        float ticks_l = p_encodeur_l->mesure();
        float ticks_r = p_encodeur_r->mesure();

        Serial.println("Encodeur gauche : " + String(ticks_l) + " ticks & " + String(ticks_l / GAIN_CM_TO_TICKS) + " cm");
        Serial.println(" | Encodeur droit : " + String(ticks_r) + " ticks & " + String(ticks_r / GAIN_CM_TO_TICKS) + " cm");
    }
}

void Pami::print_log()
{
    if (m_time_log + 500 < millis()) // Log toutes les secondes
    {
        Serial.println("Distance Ir: " + String(this->get_IR_distance()) + " mm");
        // this->print_encodeur();
        this->print_infos_interrupteur();

        m_time_log = millis();
    }
}

void Pami::print_infos_interrupteur()
{
    if (digitalRead(PIN_TIRETTE) != tirette)
    {
        tirette = digitalRead(PIN_TIRETTE);
        Serial.print(tirette == 1 ? "Tirette en place \n" : "Tirette enlevée \n");
    }
    if (digitalRead(PIN_READEQUIPE) != equipe)
    {
        equipe = digitalRead(PIN_READEQUIPE);
        Serial.print(equipe == 1 ? "Equipe : JAUNE \n" : "Equipe : BLEUE \n");
    }
    if (digitalRead(PIN_INT_PAMI_1) != int_pami_1 || digitalRead(PIN_INT_PAMI_2) != int_pami_2)
    {
        int_pami_1 = digitalRead(PIN_INT_PAMI_1);
        int_pami_2 = digitalRead(PIN_INT_PAMI_2);
        num_pami = (int_pami_1 * 2) + int_pami_2 + 1;
        Serial.print("PAMI n°" + String(num_pami) + "\n");
    }
}

/*
Avancer en ligne droite, on veut que chaque moteur avance de tick_distance ticks
*/
unsigned long Pami::avancer_asservi(int etape_d_appel, float consigne_cm, unsigned long oldtime)
{
    // Si c'est pas l'étape à laquelle on veut l'appeler,
    // aucune des variables du main n'est modifiée
    if (etape_d_appel != etape_globale)
    {
        return oldtime;
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
    if (millis() - oldtime < INTERVAL_ASSERV)
    {
        return oldtime;
    }
    else
    {
        // Sinon, c'est qu'on vient de dépasser l'invervalle d'asservissement.
        // il faut donc asservir de nouveau

        // --- Mesures actuelles ---
        float ticks_l = p_encodeur_l->mesure();
        float ticks_r = p_encodeur_r->mesure();
        // Serial.print("ticks_l : " + String(ticks_l));
        // Serial.print("ticks_r : " + String(ticks_r));

        Serial.print("Roue gauche (cm) : " + String(ticks_l / GAIN_CM_TO_TICKS));
        Serial.println("\tRoue droite (cm) : " + String(ticks_r / GAIN_CM_TO_TICKS));

        // --- Erreurs ---
        // l'erreur peut-être négative
        float erreur = ticks_l - ticks_r;

        // --- Correction --
        // si on avance
        int pwmR = SPEED + KP * erreur;
        int pwmL = SPEED - KP * erreur;
        // si on recule, on remplace les valeurs
        if (consigne_cm < 0)
        {
            pwmR = -SPEED + KP * erreur;
            pwmL = -SPEED - KP * erreur;
        }

        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);

        // Serial.print("Erreur l : ");
        // Serial.print(erreur_l_normalisee);

        // Serial.print("\tErreur : ");
        // Serial.print(erreur_normalisee);
        // Serial.print("\tpwmL : ");
        // Serial.print(pwmL);
        // Serial.print("\tpwmR : ");
        // Serial.println(pwmR);

        // --- Commande moteurs ---
        p_moteur_l->set_speed(pwmL);
        p_moteur_r->set_speed(pwmR);

        // --- Condition d’arrêt en ticks ---
        if (abs(consigne_cm * GAIN_CM_TO_TICKS - ticks_l) < MARGE_ERREUR_TICKS && abs(consigne_cm * GAIN_CM_TO_TICKS - ticks_r) < MARGE_ERREUR_TICKS)
        {
            // ON RENTRE & on nettoie les encodeurs !!
            p_moteur_l->set_speed(0);
            p_moteur_r->set_speed(0);
            p_encodeur_l->clear_count();
            p_encodeur_r->clear_count();
            etape_globale++;
        }

        // On renvoie les nouvelles valeurs
        return millis();
    }
}

// consigne angle > 0 = sens trigo
unsigned long Pami::tourner_asservi(int etape_d_appel, float consigne_angle, unsigned long oldtime)
{
    /* But du gain proportionnel : faire une correction proportionnelle à l'erreur.
    En gros :s
    erreur = ticksG - ticksD
    correction = Kp * erreur

    Puis on ajuste le pwm :
    pwmG = pwmBase - correction
    pwmD = pwmBase + correction
    */

    // Si c'est pas l'étape à laquelle on veut l'appeler,
    // aucune des variables du main n'est modifiée
    if (etape_d_appel != etape_globale)
    {
        return oldtime;
    }
    // Si l’intervalle n’est pas écoulé -> on ne fait rien
    if (millis() - oldtime < INTERVAL_ASSERV)
    {
        return oldtime;
    }
    else
    {
        // Sinon, c'est qu'on vient de dépasser l'invervalle d'asservissement.
        // il faut donc asservir de nouveau

        // --- Mesures actuelles ---
        float ticks_l = p_encodeur_l->mesure();
        float ticks_r = p_encodeur_r->mesure();

        float angle_trigo = ticks_r / GAIN_ANGLE_TO_TICKS;
        // Serial.print("ticks_l : " + String(ticks_l));
        // Serial.print("\t ticks_r : " + String(ticks_r));

        Serial.println("\t Angle en ° : " + String(ticks_r / GAIN_ANGLE_TO_TICKS));

        // --- Erreurs ---
        // On asservis le centre de gravité pour qu'il ne bouge pas,
        // Il faut que les ticks droits et gauches se compensent
        float erreur = ticks_l + ticks_r;

        // l'erreur peut-être négative,

        // --- Correction --
        // les signes ont étés vérifiés par Florian et Jules à minuit 09 mais soyez confiant, ils sont correctes !
        // Faites un schéma !
        // sens trigo
        int pwmR = SPEED - KP * erreur;
        int pwmL = -SPEED - KP * erreur;
        // sens horaire
        if (consigne_angle < 0)
        {
            pwmR = -SPEED - KP * erreur;
            pwmL = SPEED - KP * erreur;
        }

        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);

        // Serial.print("Erreur l : ");
        // Serial.print(erreur_l_normalisee);

        // Serial.print("\tErreur : ");
        // Serial.print(erreur_normalisee);
        // Serial.print("\tpwmL : ");
        // Serial.print(pwmL);
        // Serial.print("\tpwmR : ");
        // Serial.println(pwmR);

        // --- Commande moteurs ---
        p_moteur_l->set_speed(pwmL);
        p_moteur_r->set_speed(pwmR);

        // --- Condition d’arrêt en ticks ---
        // idem faites confiance ou utilisez votre cerveau
        if (abs(consigne_angle * GAIN_ANGLE_TO_TICKS + ticks_l) < MARGE_ERREUR_TICKS && abs(consigne_angle * GAIN_ANGLE_TO_TICKS - ticks_r) < MARGE_ERREUR_TICKS)
        {
            // ON RENTRE !!
            p_moteur_l->set_speed(0);
            p_moteur_r->set_speed(0);
            p_encodeur_l->clear_count();
            p_encodeur_r->clear_count();
            etape_globale++;
        }

        // On renvoie les nouvelles valeurs
        return millis();
    }
}

void Pami::go_to_asservi(float consigne_x, float consigne_y, unsigned long oldtime)
{
    switch (etape_globale)
    {
    case 0:
    {
        float consigne = 20;
        p_newtime = this->avancer_asservi(0, consigne, oldtime);
        break;
    }
    case 1:
    {
        if (millis() - oldtime > DELAY_TIME)
        {
            etape_globale = 2; // suivant
            break;
        }
    }
    case 2:
    {
        float consigne_angle = 90;
        p_newtime = this->tourner_asservi(2, consigne_angle, oldtime);
        break;
    }
    case 3:
    {
        if (millis() - oldtime > DELAY_TIME)
        {
            etape_globale = 4;
            break;
        }
    }
    case 4:
    {
        float consigne = 20;
        p_newtime = this->avancer_asservi(4, consigne, oldtime);
        break;
    }
    case 5:
    {
        if (millis() - oldtime > DELAY_TIME)
        {
            etape_globale = 6;
            break;
        }
    }
    case 6:
    {
        float consigne_angle = -180;
        p_newtime = this->tourner_asservi(6, consigne_angle, oldtime);
        break;
    }
    }
}