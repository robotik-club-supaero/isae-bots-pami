#include "esp32-hal-gpio.h"
#include <Ninja.h>
#include <define.h>

Ninja::Ninja(Moteur *p_moteur_r, Moteur *p_moteur_l, Encodeur *p_encodeur_r, Encodeur *p_encodeur_l, Serv *p_servo, Irsensor *p_ir_sensor)
{
    moteur_r = p_moteur_r;
    moteur_l = p_moteur_l;
    encodeur_r = p_encodeur_r;
    encodeur_l = p_encodeur_l;
    servo = p_servo;
    ir_sensor = p_ir_sensor;
}

/*
Avancer en ligne droite, on veut que chaque moteur avance de consigne_cm
*/
unsigned long Ninja::avancer_asservi(int etape_d_appel, float consigne_cm, unsigned long oldtime)
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
        float ticks_l = encodeur_l->mesure();
        float ticks_r = encodeur_r->mesure();
        Serial.print("ticks_l : ");
        Serial.print(ticks_l);
        Serial.print("\t ticks_r : ");
        Serial.print(ticks_r);

        // --- Erreurs ---

        float erreur = ticks_l - ticks_r;

        // l'erreur peut-être négative

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

        // Serial.print("\tErreur : ");
        // Serial.print(erreur);
        // Serial.print("\tpwmL : ");
        // Serial.print(pwmL);
        // Serial.print("\tpwmR : ");
        // Serial.println(pwmR);

        // --- Commande moteurs ---
        moteur_l->set_speed(pwmL);
        moteur_r->set_speed(pwmR);

        // --- Condition d’arrêt en ticks ---
        if (abs(consigne_cm * GAIN_CM_TO_TICKS - ticks_l) < MARGE_ERREUR_TICKS && abs(consigne_cm * GAIN_CM_TO_TICKS - ticks_r) < MARGE_ERREUR_TICKS)
        {
            // ON RENTRE !!
            moteur_l->set_speed(0);
            moteur_r->set_speed(0);
            encodeur_l->clear_count();
            encodeur_r->clear_count();
            etape_globale++;
        }

        // On renvoie les nouvelles valeurs
        return millis();
    }
}

// consigne angle > 0 = sens trigo
unsigned long Ninja::tourner_asservi(int etape_d_appel, float consigne_angle, unsigned long oldtime)
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
        float ticks_l = encodeur_l->mesure();
        float ticks_r = encodeur_r->mesure();
        Serial.print("ticks_l : ");
        Serial.print(ticks_l);
        Serial.print("\t ticks_r : ");
        Serial.println(ticks_r);

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
        moteur_l->set_speed(pwmL);
        moteur_r->set_speed(pwmR);

        // --- Condition d’arrêt en ticks ---
        // idem faites confiance ou utilisez votre cerveau
        if (abs(consigne_angle * GAIN_ANGLE_TO_TICKS + ticks_l) < MARGE_ERREUR_TICKS && abs(consigne_angle * GAIN_ANGLE_TO_TICKS - ticks_r) < MARGE_ERREUR_TICKS)
        {
            // ON RENTRE !!
            moteur_l->set_speed(0);
            moteur_r->set_speed(0);
            encodeur_l->clear_count();
            encodeur_r->clear_count();
            etape_globale++;
        }

        // On renvoie les nouvelles valeurs
        return millis();
    }
}

/*
Fonction de diagnostic général du robot
Modes :
1 = Interrupteurs & Tirette
2 = Capteur IR (ToF)
3 = Servomoteur
4 = Moteurs (Puissance brute)
5 = Encodeurs & Odométrie (À pousser à la main)
6 = Moteurs Individuels (Puissance brute)
7 = Homologation : Avance et s'arrête en fonction du capteur IR
8 = Avancer, reculer, tourner (Sans asservissement, juste pour voir si les fonctions de base marchent &  régler les gains)
*/
// void Ninja::test(int mode)
// {
//     Serial.print("\n========== LANCEMENT DU TEST MODE : ");
//     Serial.print(mode);
//     Serial.println(" ==========");

//     switch (mode)
//     {
//     case 1: // --- TEST 1 : TIRETTE & INTERRUPTEURS ---
//     {
//         Serial.println("Test Interrupteurs... Modifiez leurs etats ! (Boucle infinie)");
//         while (true)
//         {
//             Serial.println("-------------------------");
//             delay(1000); // On attend 1s pour ne pas spammer le terminal
//         }
//         break;
//     }

//     case 2: // --- TEST 2 : CAPTEUR IR ---
//     {
//         Serial.println("Test Capteur IR... Passez votre main devant ! (Boucle infinie)");
//         while (true)
//         {
//             float dist = this->get_IR_distance();
//             Serial.print("Distance mesuree : ");
//             Serial.print(dist / 10.0);
//             Serial.println(" cm");
//             delay(200);
//         }
//         break;
//     }

//     case 3: // --- TEST 3 : SERVOMOTEUR ---
//     {
//         Serial.println("Test Servomoteur : Va-et-vient de 3 secondes");
//         while (true)
//         {
//             // Utilise les constantes ANGLE1 et ANGLE2 de ton define.h
//             this->blink_servo(1000, ANGLE1, ANGLE2);
//         }
//         Serial.println("Fin du test Servomoteur.");
//         break;
//     }

//     case 4: // --- TEST 4 : MOTEURS BRUTS ---
//     {
//         Serial.println("Test Moteurs : Attention, le robot va avancer puis reculer !");
//         delay(2000); // Laisse le temps de poser le robot ou de le lever

//         Serial.println("-> Marche Avant (Vitesse SPEED)");
//         this->tout_droit(SPEED);
//         delay(1500);

//         Serial.println("-> Arret");
//         this->tout_droit(0);
//         delay(1000);

//         Serial.println("-> Marche Arriere (Vitesse -SPEED)");
//         this->tout_droit(-SPEED);
//         delay(1500);

//         Serial.println("-> Arret Definitif");
//         this->tout_droit(0);
//         Serial.println("Fin du test Moteurs.");
//         break;
//     }

//     case 5: // --- TEST 5 : ODOMETRIE ---
//     {
//         Serial.println("Test Encodeurs... Poussez le robot a la main ! (Boucle infinie)");
//         m_p_mesure_pos->reinitialise();
//         while (true)
//         {
//             m_p_mesure_pos->loop(); // Met a jour les calculs
//             this->print_encodeur();
//             this->print_position();
//             this->print_speed();
//             Serial.println("-------------------------");
//             delay(250);
//         }
//         break;
//     }
//     case 6: // Essais roue droite & gauche indépendament
//     {
//         Serial.println("Test Moteurs Individuels : Attention, le robot va tester chaque roue indépendamment !");

//         while (true)
//         {
//             Serial.println("\n-> Test Roue Droite (Vitesse 200)");
//             m_p_moteur_d->set_speed(SPEED);
//             m_p_moteur_g->set_speed(0);
//             this->print_speed();
//             this->print_encodeur();
//             delay(1500);

//             Serial.println("\n-> Arret");
//             m_p_moteur_d->set_speed(0);
//             delay(1500);

//             Serial.println("\n-> Test Roue Gauche (Vitesse 200)");
//             m_p_moteur_d->set_speed(0);
//             m_p_moteur_g->set_speed(SPEED);
//             this->print_speed();
//             this->print_encodeur();
//             delay(1500);

//             Serial.println("\n-> Arret Definitif");
//             m_p_moteur_d->set_speed(0);
//             m_p_moteur_g->set_speed(0);
//             delay(1500);
//             Serial.println("Fin du test Moteurs Individuels.");
//         }
//         break;
//     }
//     case 7: // Homologation
//     {
//         Serial.println("Homologation : Le robot avance et s'arrête lorsque le capteur IR détecte un obstacle à moins de 5 cm (Boucle infinie)");

//         while (true)
//         {
//             float dist = this->get_IR_distance(); // en mm
//             Serial.print("Distance mesuree : ");
//             Serial.print(dist / 10.0);
//             Serial.println(" cm");

//             if (dist < 120 && dist > 0.5) // Si un obstacle est détecté à moins de 8 cm
//             {
//                 Serial.println("Obstacle détecté ! Arrêt du robot.");
//                 this->tout_droit(0);
//             }
//             else
//             {
//                 this->tout_droit(SPEED);
//             }
//             delay(200);
//         }
//         break;
//     }
//     case 8:
//     {
//         Serial.println("Test Avancer/Reculer/Tourner... Attention, le robot va avancer, reculer puis tourner !");

//         this->avancer(100);
//         delay(2000);
//         // this->reculer(100);
//         delay(2000);
//         this->tourner(180);
//     }

//     default:
//     {
//         Serial.println("Erreur : Mode de test inconnu ! (Choisissez entre 1 et 6)");
//         break;
//     }
//     }
// }

/*
Fonction pour bouger le servo entre deux angles en un temps donné
*/
void Ninja::blink_servo(long temps_blink, int angle1, int angle2)
{
    servo->blink(temps_blink, angle1, angle2);
    delay(100);
}

/*
En mm
Fonction qui retourne la distance minimal au prochain obstacle détectée par le capteur infrarouge (ToF)
*/
double Ninja::get_IR_distance(unsigned long oldtime)
{
    if (ir_sensor == nullptr)
    {
        Serial.println("Pas de capteur infrarouge");
        return -1;
    }
    else
    {
        ir_sensor->loop(oldtime);
        return ir_sensor->ir_minimum_distance;
    }
}

void Ninja::print_encodeur(unsigned long oldtime)
{
    if (millis() - oldtime > 250)
    {
        Serial.print("Encodeur gauche : " + String(encodeur_l->mesure()));
        Serial.println(" | Encodeur droit : " + String(encodeur_r->mesure()));
    }
}