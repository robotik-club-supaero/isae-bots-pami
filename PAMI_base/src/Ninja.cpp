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

unsigned long Ninja::bouger_servo_non_bloquant(int etape, float angle1, float angle2)
{
    if (etape == etape_globale)
    {
        servo->blink(TEMPS_BLINK, angle1, angle2);
        etape_globale++;
        return millis();
    }
}

unsigned long Ninja::allumer_pompe(int etape)
{
    if (etape == etape_globale)
    {
        digitalWrite(PIN_POMPE, HIGH);
        etape_globale++;
        return millis();
    }
}

unsigned long Ninja::eteindre_pompe(int etape)
{
    if (etape == etape_globale)
    {
        digitalWrite(PIN_POMPE, LOW);
        etape_globale++;
        return millis();
    }
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