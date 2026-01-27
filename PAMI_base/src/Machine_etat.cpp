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

        // --- Ce qu'on fait quelquesoit l'etat de la PAMI : --- //
        m_p_ultrason->loop();
        m_minimum_distance = m_p_ultrason->m_distance; // Récupère la distance au danger le plus proche
        // ----------------------------------------------------- //

        switch (etat) // MACHINE A ETATS
        { 
        case INIT: // Etat initial, en attente du début du match

            // Lire l'état de la tirette si elle y est encore
            if (tirette == 1){ // tant que la tirette est là

                tirette = digitalRead(PIN_TIRETTE);

                m_time_global = millis();
                m_p_asserv->asserv_global(0, 0, angle); // Ne bouge pas
                
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

            if (millis() - m_time_global >= GLOBALTIME) { // Si le match est terminé (T >= 100s)
                m_p_asserv->asserv_global(0, 0, angle); // Arrêt
                etat = END;
            }
            else { // Match pas encore terminé

                if (m_minimum_distance <= DISTANCE_MIN) // Obstacle ?
                {
                    etat = STOP;
                }
                else
                {
                    pos_x = m_p_mesure_pos->position_x + pos_init_x;
                    pos_y = m_p_mesure_pos->position_y + pos_init_y;
                    
                    // -- MISE A JOUR DE L'OBJECTIF DEPLACEMENT -- //
                    // Code ICI : pos_target_x = ? & pos_target_y = ?
                    // ------------------------------------------- //
                    
                    distance_target = sqrt(pow(pos_x - pos_target_x, 2) + pow(pos_y - pos_target_y, 2)) ;
                    if (distance_target > EPSP){
                        angle = atan2(pos_target_y - pos_y, pos_target_x - pos_x);
                        m_p_asserv->asserv_global(SPEED, SPEED, angle); // corrige l'angle.
                    }

                    if (distance_target < EPSP/2) { // Fin si on est sur le target
                        etat = END;
                    }
                    else {
                        etat = MOVE; // Continue à bouger
                    }
                }
            }
            break;

        case OBSTACLE:

            if (millis() - m_time_global >= GLOBALTIME) {
                m_p_asserv->asserv_global(0, 0, angle);
                etat = END;
            } 
            else {
                if ((m_minimum_distance <= 0.1) || (m_minimum_distance >= DISTANCE_MIN)) { // Si plus d'obstacle
                    etat = MOVE;
                }
                else {
                    etat = OBSTACLE;
                }
            }
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