#include "esp32-hal-gpio.h"
#include <Pami.h>
#include <define.h>

Pami::Pami(int *p_etape_globale, Moteur *p_moteur_r, Moteur *p_moteur_l, Encodeur *p_encodeur_r, Encodeur *p_encodeur_l, Mesure_pos *p_mesure_pos, Serv *p_servo, Irsensor *p_ir_sensor, Ultrason *p_ultrason)
{
    moteur_r = p_moteur_r;
    moteur_l = p_moteur_l;
    encodeur_r = p_encodeur_r;
    encodeur_l = p_encodeur_l;
    mesure_pos = p_mesure_pos;
    servo = p_servo;
    ir_sensor = p_ir_sensor;
    etape_globale = p_etape_globale;
}


/*
Allume les deux moteurs à une vitesse en (entre 0 et 255)
*/

/*
Avancer en ligne droite, on veut que chaque moteur avance de tick_distance ticks
*/
std::tuple<float,float,unsigned long> Pami::avancer_asservi(int etape_d_appel,float consigne_cm_l, float consigne_cm_r, float old_ticks_l, float old_ticks_r, unsigned long oldtime)
{   
    // Si c'est pas l'étape à laquelle on veut l'appeler, 
    // aucune des variables du main n'est modifiée
    if (etape_d_appel != *etape_globale){
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
    if (millis() - oldtime < INTERVAL_ASSERV) {
        return std::make_tuple(old_ticks_l, old_ticks_r, oldtime);
    }
    else {
        // Sinon, c'est qu'on vient de dépasser l'invervalle d'asservissement.
        // il faut donc asservir de nouveau

        // --- Mesures actuelles ---
        float ticks_l = encodeur_l->mesure();
        float ticks_r = encodeur_r->mesure();
        // Serial.print("ticks_l : ");
        // Serial.print(ticks_l);
        // Serial.print("\t ticks_r : ");
        // Serial.print(ticks_r);

        // --- Erreurs ---

        float erreur = ticks_l - ticks_r;


        //l'erreur peut-être négative
        
        // --- Correction --
        // si on avance
        int pwmR = SPEED + KP*erreur;
        int pwmL = SPEED - KP*erreur;
        // si on recule, on remplace les valeurs
        if (consigne_cm_l < 0 || consigne_cm_r < 0){
            pwmR = - SPEED + KP*erreur;
            pwmL = - SPEED - KP*erreur;
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
        if (abs(consigne_cm_l*GAIN_CM_TO_TICKS - ticks_l) < MARGE_ERREUR_TICKS && abs(consigne_cm_r*GAIN_CM_TO_TICKS-ticks_r) < MARGE_ERREUR_TICKS ) {
            // ON RENTRE !!
            moteur_l->set_speed(0);
            moteur_r->set_speed(0);
            encodeur_l->clear_count();
            encodeur_r->clear_count();
            (*etape_globale)++;
        }

        // On renvoie les nouvelles valeurs
        return std::make_tuple(ticks_l, ticks_r, millis());
    }
}

// consigne angle > 0 = sens trigo
std::tuple<float,float,unsigned long> Pami::tourner_asservi(int etape_d_appel,float consigne_angle, float old_ticks_l, float old_ticks_r, unsigned long oldtime)
{   
    // Si c'est pas l'étape à laquelle on veut l'appeler, 
    // aucune des variables du main n'est modifiée
    if (etape_d_appel != *etape_globale){
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
    if (millis() - oldtime < INTERVAL_ASSERV) {
        return std::make_tuple(old_ticks_l, old_ticks_r, oldtime);
    }
    else {
        // Sinon, c'est qu'on vient de dépasser l'invervalle d'asservissement.
        // il faut donc asservir de nouveau

        // --- Mesures actuelles ---
        float ticks_l = encodeur_l->mesure();
        float ticks_r = encodeur_r->mesure();
        // Serial.print("ticks_l : ");
        // Serial.print(ticks_l);
        // Serial.print("\t ticks_r : ");
        // Serial.print(ticks_r);

        // --- Erreurs ---
        // On asservis le centre de gravité pour qu'il ne bouge pas,
        // Il faut que les ticks droits et gauches se compensent
        float erreur = ticks_l + ticks_r;


        //l'erreur peut-être négative,
        
        // --- Correction --
        // les signes ont étés vérifiés par Florian et Jules à minuit 09 mais soyez confiant, ils sont correctes ! 
        // Faites un schéma !
        // sens trigo
        int pwmR = SPEED - KP*erreur;
        int pwmL = -SPEED - KP*erreur;
        // sens horaire
        if (consigne_angle < 0 ){
            pwmR = - SPEED - KP*erreur;
            pwmL = SPEED - KP*erreur;
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
        if (abs(consigne_angle*GAIN_ANGLE_TO_TICKS + ticks_l) < MARGE_ERREUR_TICKS && abs(consigne_angle*GAIN_ANGLE_TO_TICKS-ticks_r) < MARGE_ERREUR_TICKS ) {
            // ON RENTRE !!
            moteur_l->set_speed(0);
            moteur_r->set_speed(0);
            encodeur_l->clear_count();
            encodeur_r->clear_count();
            (*etape_globale)++;
        }

        // On renvoie les nouvelles valeurs
        return std::make_tuple(ticks_l, ticks_r, millis());
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
// void Pami::test(int mode)
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
Tourner dans le sens trigo
*/
// void Pami::tourner(float angle_degres, float speed)
// {
//     m_p_moteur_d->set_speed(speed);
//     m_p_moteur_g->set_speed(-speed);
//     float delay_a_attendre = K_ANGLE_NAIF * (angle_degres / 360.0) * 1000;
//     delay(delay_a_attendre);
//     // if (millis()-m_time > delay_a_attendre){
//     //     this->set_speed(0);
//     //     m_time = millis();
//     // }
//     this->set_speed(0);
// }

/*
Fonction de test pour aller a une position (x, y) [mm & mm] du plateau
*/
// void Pami::go_to(float pos_final_x, float pos_final_y, int speed)
// {
//     m_p_mesure_pos->loop();
//     pos_x = m_p_mesure_pos->position_x + pos_init_x;
//     pos_y = m_p_mesure_pos->position_y + pos_init_y;
//     distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));

//     while (distance_target > EPSP)
//     {
//         // 1. Mise à jour des capteurs & de la position
//         m_p_mesure_pos->loop();
//         pos_x = m_p_mesure_pos->position_x + pos_init_x;
//         pos_y = m_p_mesure_pos->position_y + pos_init_y;

//         distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));
//         angle = atan2(pos_final_y - pos_y, pos_final_x - pos_x);

//         Serial.print("Distance target : ");
//         Serial.println(distance_target);

//         // this->print_speed();
//         // this->print_encodeur();
//         this->print_position();

//         m_p_asserv->asserv_global(speed, speed, angle);
//         delay(10);
//     }

//     // On s'arrête quand on est arrivés
//     this->set_speed(0);
// }

/*
Fonction de test pour avancer d'une certaine distance
*/
// void Pami::avancer_asserv(float distance, int speed)
// {
//     m_p_mesure_pos->loop();
//     float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
//     float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
//     float start_angle = m_p_mesure_pos->position_theta;

//     float distance_traveled = 0.0;

//     while (distance_traveled < distance)
//     {
//         m_p_mesure_pos->loop();
//         float pos_x = m_p_mesure_pos->position_x + pos_init_x;
//         float pos_y = m_p_mesure_pos->position_y + pos_init_y;

//         float dx = pos_x - start_pos_x;
//         float dy = pos_y - start_pos_y;

//         // 2. On projette ce déplacement sur l'axe du robot (produit scalaire)
//         distance_traveled = abs(dx * cos(start_angle) + dy * sin(start_angle));

//         Serial.print("distance parcourue : ");
//         Serial.println(distance_traveled);

//         Serial.println("dx : " + String(dx) + " | dy : " + String(dy));

//         m_p_asserv->asserv_global(speed, speed, start_angle);
//         delay(10);
//     }

//     // On s'arrête quand on est arrivés
//     this->set_speed(0);
// }

// /*
// Fonction de test pour reculer d'une distance en x et en y
// */
// void Pami::reculer_asserv(float distance, int speed)
// {
//     m_p_mesure_pos->loop();
//     float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
//     float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
//     float start_angle = m_p_mesure_pos->position_theta;

//     float distance_traveled = 0.0;

//     while (distance_traveled < distance)
//     {
//         m_p_mesure_pos->loop();
//         float pos_x = m_p_mesure_pos->position_x + pos_init_x;
//         float pos_y = m_p_mesure_pos->position_y + pos_init_y;

//         float dx = pos_x - start_pos_x;
//         float dy = pos_y - start_pos_y;

//         // 2. On projette ce déplacement sur l'axe du robot (produit scalaire)
//         // Comme le robot recule, cette valeur va devenir de plus en plus NÉGATIVE
//         distance_traveled = abs(dx * cos(start_angle) + dy * sin(start_angle));

//         Serial.print("Distance parcourue (à l'envers) : ");
//         Serial.println(distance_traveled);

//         m_p_asserv->asserv_global(-speed, -speed, start_angle);
//         delay(10);
//     }

//     // On s'arrête quand on est arrivés
//     this->set_speed(0);
// }

// /*
// Fonction de test qui fait tourner la pami de [theta_target] degrés
// */
// void Pami::tourner_asserv(float angle_degres, float speed)
// {
//     m_p_mesure_pos->loop();

//     float angle_rad = angle_degres * (PI / 180.0);
//     float target_theta = m_p_mesure_pos->position_theta + angle_rad;
//     float current_theta = m_p_mesure_pos->position_theta;
//     float erreur_angle = fmod(target_theta - current_theta, 2 * PI);
//     if (erreur_angle > PI)
//     {
//         erreur_angle -= 2 * PI;
//     }
//     else if (erreur_angle < -PI)
//     {
//         erreur_angle += 2 * PI;
//     }

//     while (abs(erreur_angle) > EPSA)
//     {
//         m_p_mesure_pos->loop();
//         current_theta = abs(m_p_mesure_pos->position_theta);
//         erreur_angle = fmod(target_theta - current_theta, 2 * PI);

//         Serial.print("Current angle : ");
//         Serial.println(current_theta * (180.0 / PI));
//         Serial.print("Erreur angle : ");
//         Serial.println(erreur_angle * (180.0 / PI));

//         m_p_asserv->asserv_angle(target_theta);
//         delay(10);
//     }

//     this->set_speed(0);
// }

// /*
// Fonction pour aller a une position (x, y) du plateau
// */
// bool Pami::go_to_with_obstacle(float pos_final_x, float pos_final_y, int speed)
// {
//     m_p_mesure_pos->loop();
//     pos_x = m_p_mesure_pos->position_x + pos_init_x;
//     pos_y = m_p_mesure_pos->position_y + pos_init_y;
//     distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));

//     while (distance_target > EPSP)
//     {
//         // 1. Sécurité temps de match
//         if (millis() - m_time_match >= TOTAL_TIME)
//         {
//             this->set_speed(0);
//             return false; // Fin du match, on force la sortie !
//         }

//         // 2. Distance au prochain obstacle
//         if (m_p_ir_sensor != nullptr)
//         {
//             // Condition d'évitement pour capteur ir
//             float dist_obstacle = this->get_IR_distance();

//             if (dist_obstacle > 0.1 && dist_obstacle < DISTANCE_MIN)
//             {
//                 this->set_speed(0);
//                 Serial.println("Obstacle !");
//                 delay(10);
//                 continue; // Repart au début du "do" sans avancer
//             }
//         }

//         // 3. Position actuelle
//         m_p_mesure_pos->loop();
//         pos_x = m_p_mesure_pos->position_x + pos_init_x;
//         pos_y = m_p_mesure_pos->position_y + pos_init_y;

//         // 4. Déplacement
//         distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));
//         angle = atan2(pos_final_y - pos_y, pos_final_x - pos_x);
//         angle = fmod(angle, 2 * PI);
//         if (angle > PI)
//             angle -= 2 * PI;
//         else if (angle < -PI)
//             angle += 2 * PI;

//         float erreur_angle = angle - m_p_mesure_pos->position_theta;
//         erreur_angle = fmod(erreur_angle, 2 * PI);
//         if (erreur_angle > PI)
//             erreur_angle -= 2 * PI;
//         else if (erreur_angle < -PI)
//             erreur_angle += 2 * PI;

//         Serial.print("Distance target : ");
//         Serial.println(distance_target);
//         Serial.print("erreur_angle : ");
//         Serial.println(erreur_angle);

//         // Si l'angle est trop éloigné, on tourne sur place avant d'avancer.
//         if (abs(erreur_angle) > 0.25)
//         {
//             m_p_asserv->asserv_global(0, 0, angle);
//         }
//         else
//         {
//             m_p_asserv->asserv_global(speed, speed, angle);
//         }
//         delay(10);
//     }

//     // On s'arrête quand on est arrivés
//     this->set_speed(0);
//     return true;
// }

/*
Fonction pour avancer d'une certaine distance
*/
// bool Pami::avancer_with_obstacle(float distance, int speed)
// {
//     m_p_mesure_pos->loop();
//     float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
//     float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
//     float start_angle = m_p_mesure_pos->position_theta;

//     float distance_parcourue = 0.0;

//     while (distance_parcourue < distance)
//     {
//         // 1. Sécurité temps de match
//         if (millis() - m_time_match >= TOTAL_TIME)
//         {
//             m_p_asserv->asserv_global(0, 0, start_angle);
//             return false;
//         }

//         // 2. Distance au prochain obstacle
//         if (m_p_ir_sensor != nullptr)
//         {
//             float dist_obstacle = this->get_IR_distance();

//             if (dist_obstacle > 0.01 && dist_obstacle < DISTANCE_MIN)
//             {
//                 this->set_speed(0);
//                 // m_p_asserv->asserv_global(0, 0, start_angle);
//                 Serial.println("Obstacle !");
//                 delay(10);
//                 continue; // Repart au début du "do" sans avancer
//             }
//         }

//         // 3. Position actuelle
//         m_p_mesure_pos->loop();
//         float pos_x = m_p_mesure_pos->position_x + pos_init_x;
//         float pos_y = m_p_mesure_pos->position_y + pos_init_y;

//         // 4. Déplacement
//         float dx = pos_x - start_pos_x;
//         float dy = pos_y - start_pos_y;

//         distance_parcourue = dx * cos(start_angle) + dy * sin(start_angle);

//         Serial.print("Distance parcourue : ");
//         Serial.println(distance_parcourue);

//         m_p_asserv->asserv_global(speed, speed, start_angle);
//         delay(10);
//     }

//     // On s'arrête quand on est arrivés
//     this->set_speed(0);
//     return true;
// }

// /*
// Fonction pour reculer d'une distance en x et en y
// */
// bool Pami::reculer_with_obstacle(float distance, int speed)
// {
//     m_p_mesure_pos->loop();
//     float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
//     float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
//     float start_angle = m_p_mesure_pos->position_theta;

//     float distance_parcourue = 0.0;

//     while (distance_parcourue < distance)
//     {
//         // 1. Sécurité temps de match
//         if (millis() - m_time_match >= TOTAL_TIME)
//         {
//             m_p_asserv->asserv_global(0, 0, start_angle);
//             return false;
//         }

//         // 2. Distance au prochain obstacle
//         if (m_p_ir_sensor != nullptr)
//         {
//             float dist_obstacle = this->get_IR_distance();

//             if (dist_obstacle > 0.1 && dist_obstacle < DISTANCE_MIN)
//             {
//                 m_p_asserv->asserv_global(0, 0, start_angle);
//                 Serial.println("Obstacle !");
//                 delay(10);
//                 continue; // Repart au début du "do" sans avancer
//             }
//         }

//         // 3. Mise à jour des capteurs
//         m_p_mesure_pos->loop();
//         float pos_x = m_p_mesure_pos->position_x + pos_init_x;
//         float pos_y = m_p_mesure_pos->position_y + pos_init_y;

//         // 4. Déplacement
//         float dx = pos_x - start_pos_x;
//         float dy = pos_y - start_pos_y;

//         distance_parcourue = abs(dx * cos(start_angle) + dy * sin(start_angle));

//         Serial.print("Distance parcourue (à l'envers) : ");
//         Serial.println(distance_parcourue);

//         m_p_asserv->asserv_global(-speed, -speed, start_angle);
//         delay(10);
//     }

//     // On s'arrête quand on est arrivés
//     this->set_speed(0);
//     return true;
// }






/*
Fonction pour bouger le servo entre deux angles en un temps donné
*/
// void Pami::blink_servo(long temps_blink, int angle1, int angle2)
// {
    // m_p_servo->blink(temps_blink, angle1, angle2);
    // delay(100);
// }
// 
/*
Fonction qui affiche la distance au prochain obstacle détectée par le capteur ultrason
*/
// double Pami::get_ultrason_distance()
// {
//     if (m_p_ultrason == nullptr)
//     {
//         Serial.println("Pas de capteur ultrason");
//         return -1;
//     }
//     else
//     {
//         return m_p_ultrason->m_distance;
//     }
// }

/*
En mm
Fonction qui retourne la distance minimal au prochain obstacle détectée par le capteur infrarouge (ToF)
*/
// double Pami::get_IR_distance()
// {
//     if (m_p_ir_sensor == nullptr)
//     {
//         Serial.println("Pas de capteur infrarouge");
//         return -1;
//     }
//     else
//     {
//         m_p_ir_sensor->loop();
//         return m_p_ir_sensor->ir_minimum_distance;
//     }
// }

// void Pami::print_position()
// {
//     if (millis() - m_time > 250)
//     {
//         m_p_mesure_pos->loop();
//         Serial.print("Pos X : " + String(m_p_mesure_pos->position_x / 10) + " cm");
//         Serial.print(" | Pos Y : " + String(m_p_mesure_pos->position_y / 10) + " cm");
//         Serial.println(" | Theta : " + String(m_p_mesure_pos->position_theta * (180.0 / PI)) + "°");
//     }
// }

// void Pami::print_encodeur()
// {
//     if (millis() - m_time > 250)
//     {
//         Serial.print("Encodeur gauche : " + String(m_p_encodeur_g->mesure()));
//         Serial.println(" | Encodeur droit : " + String(m_p_encodeur_d->mesure()));
//     }
// }

// void Pami::print_speed()
// {
//     if (millis() - m_time > 275)
//     {
//         m_p_mesure_pos->loop();
//         Serial.print("Vitesse droite : " + String(m_p_mesure_pos->vitesse_r / 10) + " cm/s | Vitesse gauche : " + String(m_p_mesure_pos->vitesse_l / 10) + " cm/s");
//         Serial.print(" | Vitesse en x : " + String(m_p_mesure_pos->vitesse_x / 10) + " cm/s | Vitesse en y : " + String(m_p_mesure_pos->vitesse_y / 10) + " cm/s");
//         Serial.println(" | Vitesse angulaire : " + String(m_p_mesure_pos->vitesse_theta) + " rad/s");
//     }
// }

// void Pami::print_log()
// {
//     if (m_time + 500 < millis()) // Log toutes les secondes
//     {
//         Serial.println("Distance Ir: " + String(this->get_IR_distance()) + " mm");
//         this->print_speed();
//         this->print_position();
//         this->print_encodeur();

//         m_time = millis();
//     }
// }


// void Pami::start_match()
// {
//     // --- 1. PHASE INIT (Attente Tirette) ---
//     Serial.println("Attente de la tirette...");
//     while (digitalRead(PIN_TIRETTE) == 1)
//     {
//         this->set_speed(0);
//         delay(10);
//     }

//     // --- 2. DEPART (La tirette est tirée) ---
//     Serial.println("Début du match");
//     m_time_match = millis(); // On lance le chrono de 100s

//     // On attend le délai de départ de la PAMI (ex: démarre à T+85s)
//     while (millis() - m_time_match < START_TIME)
//     {
//         delay(10);
//     }

//     if (num_pami == 1)
//     {
//         if (equipe == 1)
//         {
//             if (go_to_with_obstacle(J_POSITION_1_FINAL_X, J_POSITION_1_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//         else
//         {
//             if (go_to_with_obstacle(B_POSITION_1_FINAL_X, B_POSITION_1_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//     }
//     else if (num_pami == 2)
//     {
//         if (equipe == 1)
//         {
//             if (go_to_with_obstacle(J_POSITION_2_FINAL_X, J_POSITION_1_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//         else
//         {
//             if (go_to_with_obstacle(B_POSITION_2_FINAL_X, B_POSITION_1_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//     }
//     else if (num_pami == 3)
//     {
//         if (equipe == 1)
//         {
//             if (go_to_with_obstacle(J_POSITION_3_FINAL_X, J_POSITION_3_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//         else
//         {
//             if (go_to_with_obstacle(B_POSITION_3_FINAL_X, B_POSITION_3_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//     }
//     else if (num_pami == 4)
//     {
//         if (equipe == 1)
//         {
//             if (go_to_with_obstacle(J_POSITION_4_FINAL_X, J_POSITION_1_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//         else
//         {
//             if (go_to_with_obstacle(B_POSITION_4_FINAL_X, B_POSITION_1_FINAL_Y, SPEED))
//                 this->end_match();
//         }
//     }
// }

// void Pami::end_match()
// {
//     Serial.println("Fin du match (Temps écoulé !)");
//     this->set_speed(0);

//     while (true)
//     {
//         this->blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
//         delay(100);
//     }
// }