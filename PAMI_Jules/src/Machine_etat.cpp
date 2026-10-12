#include <Machine_etats.h>
#include <Arduino.h>
#include <cmath>

#include <define.h>

Machine_etats::Machine_etats(Asserv *p_asserv, Mesure_pos *p_mesure_pos, Ultrason *p_ultrason)
{
    m_p_asserv = p_asserv;
    m_p_mesure_pos = p_mesure_pos;
    m_p_ultrason = p_ultrason;
}

void Machine_etats::setup()
{
    pinMode(PIN_TIRETTE, INPUT);
    pinMode(PIN_READEQUIPE, INPUT);
    etat = INIT; // Etat initial de la PAMI
    m_time = millis();
    m_time_global = millis();
}

void Machine_etats::loop()
{

    if (millis() - m_time >= dt) // Tout les dt
    {
        Serial.print("Etat ");
        Serial.println(etat);
        // --- Ce qu'on fait quelquesoit l'etat de la PAMI : --- //
        // m_p_ultrason->loop();
        // m_minimum_distance = m_p_ultrason->m_distance; // Récupère la distance au danger le plus proche
        // ----------------------------------------------------- //

        switch (etat) // MACHINE A ETATS
        { 
        case INIT: // Etat initial, en attente du début du match
            // Lire l'état de la tirette si elle y est encore
            if (tirette == 1){ // tant que la tirette est là
                Serial.println("Attente enlevage tirette");
                tirette = digitalRead(PIN_TIRETTE);

                m_time_global = millis();
                m_p_asserv->asserv_global(0, 0, 0); // Ne bouge pas et regarde en face
                
                // Lis dans quelle équipe est la pami (interrupteur)
                int read_equipe = digitalRead(PIN_READEQUIPE);
                if (read_equipe != equipe){ // Si l'etat de l'interrupteur à changé on mets a jour les variables
                    equipe = read_equipe ;
                    if (equipe == 1){
                        pos_init_x = J_POSITION_DEPART_X;
                        pos_init_y = J_POSITION_DEPART_Y;
                        pos_target_x = J_POSITION_FINAL_X; 
                        pos_target_y = J_POSITION_FINAL_Y; 
                    } else {
                        pos_init_x = B_POSITION_DEPART_X;
                        pos_init_y = B_POSITION_DEPART_Y;
                        pos_target_x = B_POSITION_FINAL_X; 
                        pos_target_y = B_POSITION_FINAL_Y; 
                    }
                }

            } else { // La tirette à été enlevé
                if ((millis() - m_time_global) >= START_TIME) { // En attente de  temps de go
                    Serial.println("PAMI GO !");
                    etat = MOVE;
                }
                else {
                    Serial.print("Temps écoulé : ");
                    Serial.print(millis() - m_time_global);
                    Serial.print("Temps avant départ : ");
                    Serial.print(millis() - m_time_global - START_TIME);
                    etat = INIT; // On ne change rien
                }
            }
            break;
        case MOVE:
            Serial.println("Etat Move");
            if (millis() - m_time_global >= GLOBALTIME) { // Si le match est terminé (T >= 100s)
                m_p_asserv->asserv_global(0, 0, angle); // Arrêt
                etat = END;
            }
            else { // Match pas encore terminé
                Serial.println("Match pas encore terminé");
                

                if (false){ //pour remplacer le if
                    // if (m_minimum_distance <= DISTANCE_MIN) // Obstacle
                // {
                //     Serial.println("Obstacle détecté");
                //     etat = OBSTACLE;
                // }
                }
                
                else
                {
                    pos_x = m_p_mesure_pos->position_x + pos_init_x;
                    pos_y = m_p_mesure_pos->position_y + pos_init_y;
                    Serial.print("x,y = ");
                    Serial.print(pos_x);
                    Serial.print(",");
                    Serial.println(pos_y);

                    
                    // -- MISE A JOUR DE L'OBJECTIF DEPLACEMENT -- //
                    // Code ICI : pos_target_x = ? & pos_target_y = ?
                    // ------------------------------------------- //
                    pos_target_x = 100;
                    pos_target_y = 100;
                    
                    distance_target = sqrt(pow(pos_x - pos_target_x, 2) + pow(pos_y - pos_target_y, 2)) ;
                    Serial.print("Distance_target : ");
                    Serial.println(distance_target);
                    if (distance_target > EPSP){
                        angle = atan2(pos_target_y - pos_y, pos_target_x - pos_x);
                        Serial.print("Angle : ");
                        Serial.print(angle*180/PI);
                        Serial.println("°");
                        m_p_asserv->asserv_global(SPEED, SPEED, angle); // corrige l'angle.
                    }
                    else { // Fin si on est sur le target
                        Serial.println("On est sur le target");
                        etat = END;
                    }
                }
            }
            break;

        case OBSTACLE:
            etat = MOVE;
            // if (millis() - m_time_global >= GLOBALTIME) {
            //     m_p_asserv->asserv_global(0, 0, angle);
            //     etat = END;
            // } 
            // else {
            //     int distance_prev =  m_p_ultrason->m_distance;
            //     int distance_curr =  m_p_ultrason->m_distance;
            //     bool contournement = true;
            //     while (contournement){
            //         // choix d'avancer ou non
            //         int speed = distance_curr >= distance_prev ? SPEED : - SPEED;
            //         // on bouge
            //         m_p_asserv->asserv_global(speed,speed, angle);
            //         delay(500);
            //         m_p_asserv->asserv_global(0, 0, angle);

            //         int direction_vers_goal = atan2(pos_target_y - pos_y, pos_target_x - pos_x);
            //         int delta_angle = direction_vers_goal < angle ? -5:5;
            //         angle += delta_angle * PI / 360;

            //         distance_prev = distance_curr;
            //         distance_curr = m_p_ultrason->m_distance;
            //         if (distance_curr >= DISTANCE_MIN){
            //             contournement = false;
            //         }
            //     }
            //     etat = MOVE;
            // }
            break;

        case STOP: 

            if (millis() - m_time_global >= GLOBALTIME) {
                m_p_asserv->asserv_global(0, 0, angle);
                etat = END;
            }
            else {
                m_p_asserv->asserv_global(0, 0, m_p_mesure_pos->position_theta); // Stop le mouvement
            }
            break;

        case END: // Stop tout

            m_p_asserv->asserv_global(0, 0, m_p_mesure_pos->position_theta);
            break;
        }

        m_time = millis();
    }
}