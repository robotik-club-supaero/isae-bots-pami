#include <Machine_etats.h>
#include <Arduino.h>
#include <cmath>

#include <define.h>


double distance(const Point& a, const Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

bool isAtTarget(const Point& currentPos, const Point& target, double threshold) {
    return distance(currentPos, target) < threshold;
}

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
    
    // DEMO
    targetIndex = 0 ;

}

void Machine_etats::loop(){

    if (millis() - m_time >= dt) // Tout les dt
    {

        // --- Ce qu'on fait quelquesoit l'etat de la PAMI : --- //
        m_p_ultrason->loop();
        m_minimum_distance = m_p_ultrason->m_distance; // Récupère la distance au danger le plus proche
        // ----------------------------------------------------- //

        switch (etat) // MACHINE A ETATS
        { 
        case INIT: // Etat initial, en attente du début du match
            //Serial.println("ETAT : INIT");

            // Lire l'état de la tirette si elle y est encore
            if (tirette == 1){ // tant que la tirette est là

                tirette = digitalRead(PIN_TIRETTE); // Interrupteur ou tirette
                
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
                    Serial.println(millis() - m_time_global);
                    Serial.print("Temps avant départ : ");
                    Serial.println(millis() - m_time_global - START_TIME);
                    etat = INIT; // On ne change rien
                }
            }
            break;
        case MOVE:
            //Serial.println("ETAT : MOVE");

            if (millis() - m_time_global >= GLOBALTIME) { // Si le match est terminé (T >= 100s)
                m_p_asserv->asserv_global(0, 0, angle); // Arrêt
                etat = END;
            }
            else { // Match pas encore terminé

                if (m_minimum_distance <= DISTANCE_MIN) // Obstacle ?
                {
                    etat = OBSTACLE;
                }
                else
                {
                    pos_x = m_p_mesure_pos->position_x + pos_init_x;
                    pos_y = m_p_mesure_pos->position_y + pos_init_y;

                    // -- MISE A JOUR DE L'OBJECTIF DEPLACEMENT -- //
                    // Code ICI : pos_target_x = ? & pos_target_y = ?
                    // ------------------------------------------- //

                    Point position{pos_x, pos_y};
                    
                    // If checkpoint reached -> next checkpoint
                    if (isAtTarget(position, checkpoints[targetIndex])) {
                        Serial.println("Next Target !");
                        targetIndex++;
                        if (targetIndex == checkpoints.size()) {
                            if (checkpoints_loop == 1){
                                targetIndex = 0 ;
                            } else {
                                etat = END ;
                                return ;
                            }
                        }
                    }
                    
                    // Go to checkpoint
                    target = checkpoints[targetIndex] ;
                    angle = fmod(atan2(target.y - pos_y, target.x - pos_x), 2 * PI);
                    
                    int speed = SPEED ;
                    if (std::abs(angle_diff(angle, m_p_mesure_pos->position_theta)) > PI){
                        speed = -1 * speed ;
                    }
                    
                    if (std::abs(angle_diff(angle, m_p_mesure_pos->position_theta)) < 0.2){
                        m_p_asserv->asserv_global(speed, speed, angle); // se dirige en direction de l'angle et avance.
                    } else {
                        m_p_asserv->asserv_global(speed/4., speed/4., angle); // se dirige en direction de l'angle.
                    }

                    etat = MOVE ; // Stay in MOVE
                    return ;
                }
            }
            break;

        case OBSTACLE:
            //Serial.println("ETAT : OBSTACLE (à " + String(m_minimum_distance) + " cm)");
            
            if (millis() - m_time_global >= GLOBALTIME) {
                m_p_asserv->asserv_global(0, 0, m_p_mesure_pos->position_theta);
                etat = END;
            }
            else {
                if (m_minimum_distance >= DISTANCE_MIN) { // Si plus d'obstacle
                    etat = MOVE;
                }
                else {
                    etat = OBSTACLE;
                }
            }
            break;

        case STOP: 
            Serial.println("ETAT : STOP");

            if (millis() - m_time_global >= GLOBALTIME) {
                m_p_asserv->asserv_global(0, 0, angle);
                etat = END;
            }
            else {
                m_p_asserv->asserv_global(0, 0, m_p_mesure_pos->position_theta); // Stop le mouvement
            }
            break;

        case END: // Stop tout
            Serial.println("ETAT : END");

            m_p_asserv->asserv_global(0, 0, m_p_mesure_pos->position_theta);
            break;
        }

        m_time = millis();
    }
}