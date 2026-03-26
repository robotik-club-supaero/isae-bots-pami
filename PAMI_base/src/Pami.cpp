#include <Pami.h>
#include <cmath>

Pami::Pami(Moteur *p_moteur_d, Moteur *p_moteur_g, Encodeur *p_encodeur_d, Encodeur *p_encodeur_g, Mesure_pos *p_mesure_pos, Serv *p_servo, Asserv *p_asserv, Irsensor *p_ir_sensor, Ultrason *p_ultrason)
{
    m_p_moteur_d = p_moteur_d;
    m_p_moteur_g = p_moteur_g;
    m_p_encodeur_d = p_encodeur_d;
    m_p_encodeur_g = p_encodeur_g;
    m_p_mesure_pos = p_mesure_pos;
    m_p_servo = p_servo;
    m_p_asserv = p_asserv;

    // Optionnels
    m_p_ultrason = p_ultrason;
    m_p_ir_sensor = p_ir_sensor;
}

float m_time_match = millis();

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
        Serial.print("Nouvelle Equipe : ");
        Serial.println(color_equipe);

        Serial.print("PAMI n°");
        Serial.println(num_pami);

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

void Pami::setup()
{
    Serial.begin(115200); // Initialisation de la communication série
    Serial.println("---------- Setup starting ----------");

    // Setup ultrason
    if (m_p_ultrason != nullptr)
    {
        m_p_ultrason->setup();
        Serial.println("Setup Done : Ultrason");
    }

    // Setup capteur IR
    if (m_p_ir_sensor != nullptr)
    {
        m_p_ir_sensor->setup();
        Serial.println("Setup Done : IR Sensor");
    }

    // Setup servo
    m_p_servo->setup();
    Serial.println("Setup Done : Servo");

    // Setup mesure position
    m_p_mesure_pos->setup();
    Serial.println("Setup Done : Mesure de Position");

    // Setup moteur droit & gauche
    m_p_moteur_d->setup();
    m_p_moteur_g->setup();
    Serial.println("Setup Done : Moteurs");

    // Setup asservissement
    m_p_asserv->setup();
    Serial.println("Setup Done : Asservissement");

    // pinMode(LED, OUTPUT);
    // digitalWrite(LED, 1); // LED ON pour indiquer le setup réussi

    pinMode(PIN_TIRETTE, INPUT);
    pinMode(PIN_READEQUIPE, INPUT);
    // pinMode(PIN_INT_PAMI_1, INPUT);
    // pinMode(PIN_INT_PAMI_2, INPUT);

    this->config_start_position();
    Serial.println("Setup Done : Tirette & Equipe & PAMI");

    m_time_log = millis();

    Serial.println("\n---------- Setup over ----------\n\n");
}

/*
Fonction de test pour aller a une position (x, y) du plateau
*/
void Pami::go_to(float pos_final_x, float pos_final_y, int speed)
{
    m_p_mesure_pos->loop();
    pos_x = m_p_mesure_pos->position_x + pos_init_x;
    pos_y = m_p_mesure_pos->position_y + pos_init_y;
    distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));

    while (distance_target > EPSP)
    {
        // 1. Mise à jour des capteurs & de la position
        m_p_mesure_pos->loop();
        pos_x = m_p_mesure_pos->position_x + pos_init_x;
        pos_y = m_p_mesure_pos->position_y + pos_init_y;

        distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));
        angle = atan2(pos_final_y - pos_y, pos_final_x - pos_x);

        Serial.print("Distance target : ");
        Serial.println(distance_target);

        // this->print_speed();
        // this->print_position();

        m_p_asserv->asserv_global(speed, speed, angle);
        delay(10);
    }

    // On s'arrête quand on est arrivés
    this->allumer_moteur(0);
}

/*
Fonction de test pour avancer d'une certaine distance
*/
void Pami::avancer(float distance, int speed)
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

        Serial.print("distance parcourue : ");
        Serial.println(distance_traveled);

        m_p_asserv->asserv_global(speed, speed, start_angle);
        delay(10);
    }

    // On s'arrête quand on est arrivés
    this->allumer_moteur(0);
}

/*
Fonction de test pour reculer d'une distance en x et en y
*/
void Pami::reculer(float distance, int speed)
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
    this->allumer_moteur(0);
}

/*
Fonction de test qui fait tourner la pami de [theta_target] degrés
*/
void Pami::tourner(float angle_degres)
{
    m_p_mesure_pos->loop();
    float angle_rad = angle_degres * (PI / 180.0);
    float target_theta = m_p_mesure_pos->position_theta + angle_rad;
    float erreur_angle = angle_rad;

    while (abs(erreur_angle) > EPSA)
    {
        m_p_mesure_pos->loop();
        float current_theta = m_p_mesure_pos->position_theta;

        // Calcul de l'erreur avec le modulo (pour trouver le chemin le plus court)
        erreur_angle = target_theta - current_theta;
        erreur_angle = fmod(erreur_angle, 2 * PI);
        if (erreur_angle > PI)
        {
            erreur_angle -= 2 * PI;
        }
        else if (erreur_angle < -PI)
        {
            erreur_angle += 2 * PI;
        }

        Serial.print("Target angle : ");
        Serial.println(target_theta);
        Serial.print("Current angle : ");
        Serial.println(current_theta);
        Serial.print("Erreur angle : ");
        Serial.println(erreur_angle);

        m_p_asserv->asserv_angle(target_theta);
        delay(10);
    }

    // On s'arrête une fois fini
    this->allumer_moteur(0);
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
        // 1. Sécurité temps de match
        if (millis() - m_time_match >= GLOBALTIME)
        {
            this->allumer_moteur(0);
            return false; // Fin du match, on force la sortie !
        }

        // 2. Distance au prochain obstacle
        if (m_p_ir_sensor != nullptr)
        {
            // Condition d'évitement pour capteur ir
            float dist_obstacle = this->get_IR_distance();

            if (dist_obstacle > 0.1 && dist_obstacle < DISTANCE_MIN)
            {
                this->allumer_moteur(0);
                Serial.println("Obstacle !");
                delay(10);
                continue; // Repart au début du "do" sans avancer
            }
        }

        // 3. Position actuelle
        m_p_mesure_pos->loop();
        pos_x = m_p_mesure_pos->position_x + pos_init_x;
        pos_y = m_p_mesure_pos->position_y + pos_init_y;

        // 4. Déplacement
        distance_target = sqrt(pow(pos_x - pos_final_x, 2) + pow(pos_y - pos_final_y, 2));
        angle = atan2(pos_final_y - pos_y, pos_final_x - pos_x);

        Serial.print("Distance target : ");
        Serial.println(distance_target);

        // this->print_speed();
        // this->print_position();

        m_p_asserv->asserv_global(speed, speed, angle);
        delay(10);
    }

    // On s'arrête quand on est arrivés
    this->allumer_moteur(0);
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
        if (millis() - m_time_match >= GLOBALTIME)
        {
            m_p_asserv->asserv_global(0, 0, start_angle);
            return false;
        }

        // 2. Distance au prochain obstacle
        if (m_p_ir_sensor != nullptr)
        {
            float dist_obstacle = this->get_IR_distance();

            if (dist_obstacle > 0.1 && dist_obstacle < DISTANCE_MIN)
            {
                m_p_asserv->asserv_global(0, 0, start_angle);
                Serial.println("Obstacle !");
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
    this->allumer_moteur(0);
    return true;
}

/*
Fonction pour reculer d'une distance en x et en y
*/
bool Pami::reculer_with_obstacle(float distance, int speed)
{
    m_p_mesure_pos->loop();
    float start_pos_x = m_p_mesure_pos->position_x + pos_init_x;
    float start_pos_y = m_p_mesure_pos->position_y + pos_init_y;
    float start_angle = m_p_mesure_pos->position_theta;

    float distance_parcourue = 0.0;

    while (distance_parcourue < distance)
    {
        // 1. Sécurité temps de match
        if (millis() - m_time_match >= GLOBALTIME)
        {
            m_p_asserv->asserv_global(0, 0, start_angle);
            return false;
        }

        // 2. Distance au prochain obstacle
        if (m_p_ir_sensor != nullptr)
        {
            float dist_obstacle = this->get_IR_distance();

            if (dist_obstacle > 0.1 && dist_obstacle < DISTANCE_MIN)
            {
                m_p_asserv->asserv_global(0, 0, start_angle);
                Serial.println("Obstacle !");
                delay(10);
                continue; // Repart au début du "do" sans avancer
            }
        }

        // 3. Mise à jour des capteurs
        m_p_mesure_pos->loop();
        float pos_x = m_p_mesure_pos->position_x + pos_init_x;
        float pos_y = m_p_mesure_pos->position_y + pos_init_y;

        // 4. Déplacement
        float dx = pos_x - start_pos_x;
        float dy = pos_y - start_pos_y;

        distance_parcourue = abs(dx * cos(start_angle) + dy * sin(start_angle));

        Serial.print("Distance parcourue (à l'envers) : ");
        Serial.println(distance_parcourue);

        m_p_asserv->asserv_global(-speed, -speed, start_angle);
        delay(10);
    }

    // On s'arrête quand on est arrivés
    this->allumer_moteur(0);
    return true;
}

/*
Modifie la position initiale en x et en y du robot
*/
void Pami::set_initial_position(float pos_initial_x, float pos_initial_y)
{
    pos_init_x = pos_initial_x;
    pos_init_y = pos_initial_y;
}

/*
Allume les deux moteurs à une vitesse en (entre 0 et 255)
*/
void Pami::allumer_moteur(float speed)
{
    // Si on règle les gains askip c'est mieux
    // m_p_asserv->asservissement(speed, speed);
    m_p_moteur_d->set_speed(speed);
    m_p_moteur_g->set_speed(speed);
    delay(100);
}

/*
Fonction pour bouger le servo entre deux angles en un temps donné
*/
void Pami::blink_servo(long temps_blink, int angle1, int angle2)
{
    m_p_servo->blink(temps_blink, angle1, angle2);
    delay(100);
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
        Serial.print("Pos X :");
        Serial.print(m_p_mesure_pos->position_x);
        Serial.print(" |  Pos Y :");
        Serial.print(m_p_mesure_pos->position_y);
        Serial.print(" | Theta :");
        Serial.println(m_p_mesure_pos->position_theta);
    }
}

void Pami::print_encodeur()
{
    if (millis() - m_time_log > 250)
    {
        Serial.print("Encodeur droit : ");
        Serial.print(m_p_encodeur_d->mesure());
        Serial.print(" | Encodeur gauche : ");
        Serial.println(m_p_encodeur_g->mesure());
    }
}

void Pami::print_speed()
{
    if (millis() - m_time_log > 275)
    {
        m_p_mesure_pos->loop();
        Serial.print("Vitesse droite : ");
        Serial.print(m_p_mesure_pos->vitesse_r);
        Serial.print(" cm/s | Vitesse gauche : ");
        Serial.print(m_p_mesure_pos->vitesse_l);
        Serial.println(" cm/s");
    }
}

void Pami::print_log()
{
    if (m_time_log + 500 < millis()) // Log toutes les secondes
    {
        Serial.print("Distance Ir: ");
        Serial.println(this->get_IR_distance());
        this->print_speed();
        this->print_position();
        this->print_encodeur();

        m_time_log = millis();
    }
}

void Pami::start_match()
{
    // --- 1. PHASE INIT (Attente Tirette) ---
    Serial.println("Attente de la tirette...");
    while (digitalRead(PIN_TIRETTE) == 1)
    {
        this->config_start_position();
        this->allumer_moteur(0);
        delay(10);
    }

    // --- 2. DEPART (La tirette est tirée) ---
    Serial.println("Début du match");
    m_time_match = millis(); // On lance le chrono de 100s

    // On attend le délai de départ de la PAMI (ex: démarre à T+85s)
    while (millis() - m_time_match < START_TIME)
    {
        delay(10);
    }

    if (num_pami == 1)
    {
        if (equipe == 1)
        {
            if (go_to_with_obstacle(J_POSITION_1_FINAL_X, J_POSITION_1_FINAL_Y, SPEED))
                this->end_match();
        }
        else
        {
            if (go_to_with_obstacle(B_POSITION_1_FINAL_X, B_POSITION_1_FINAL_Y, SPEED))
                this->end_match();
        }
    }
    else if (num_pami == 2)
    {
        if (equipe == 1)
        {
            if (go_to_with_obstacle(J_POSITION_2_FINAL_X, J_POSITION_1_FINAL_Y, SPEED))
                this->end_match();
        }
        else
        {
            if (go_to_with_obstacle(B_POSITION_2_FINAL_X, B_POSITION_1_FINAL_Y, SPEED))
                this->end_match();
        }
    }
    else if (num_pami == 3)
    {
        if (equipe == 1)
        {
            if (go_to_with_obstacle(J_POSITION_3_FINAL_X, J_POSITION_3_FINAL_Y, SPEED))
                this->end_match();
        }
        else
        {
            if (go_to_with_obstacle(B_POSITION_3_FINAL_X, B_POSITION_3_FINAL_Y, SPEED))
                this->end_match();
        }
    }
    else if (num_pami == 4)
    {
        if (equipe == 1)
        {
            if (go_to_with_obstacle(J_POSITION_4_FINAL_X, J_POSITION_1_FINAL_Y, SPEED))
                this->end_match();
        }
        else
        {
            if (go_to_with_obstacle(B_POSITION_4_FINAL_X, B_POSITION_1_FINAL_Y, SPEED))
                this->end_match();
        }
    }
}

void Pami::end_match()
{
    Serial.println("Fin du match (Temps écoulé !)");
    this->allumer_moteur(0);

    while (true)
    {
        this->blink_servo(TEMPS_BLINK, ANGLE1, ANGLE2);
        delay(100);
    }
}