/**
 * @file define.h
 * @brief fichier de configuration des pins pour la pami )
 */

// TODO REGLER LES PINS AVANT LA COUPE AVEC L'ELEC
#ifndef DEFINE_H
#define DEFINE_H

/*
MOTEURS
*/
#define EN_L 27  // M1 EN LEFT MOTEUR
#define IN1_L 26 // M1 IN1 LEFT MOTEUR
#define IN2_L 25 // M1 IN2 LEFT MOTEUR

#define EN_R 33  // M2 EN RIGHT MOTEUR
#define IN1_R 32 // M2 IN1 RIGHT MOTEUR
#define IN2_R 14 // M2 IN2 RIGHT MOTEUR

/*
ENCODEURS
*/
#define CLK_L 36 // C1 LEFT ENCODER
#define DT_L 39  // C2 LEFT ENCODER

#define CLK_R 34 // C1 RIGHT ENCODER
#define DT_R 35  // C2 RIGHT ENCODER

/*
A tester avec print_vitesse ou print_encodeur pour que les vitesses soient positives et que le robot avance
    Mais modifier le sens moteur change le sens encodeur
    Mais modifier le sens encodeur ne change pas le sens moteur
# Mention abominable
*/
#define INV_ENC_R 1 // Inversion du sens de rotation de l'encodeur gauche
#define INV_MOT_R 0 // Inversion du sens de rotation du moteur gauche

#define INV_ENC_L 0 // Inversion du sens de rotation de l'encodeur droit
#define INV_MOT_L 1 // Inversion du sens de rotation du moteur gauche

/*
SENSORS (IR, Ultrason, ect...)
L'IR Sensor demande :
    - PIN I2C SCL & SDA définits dessous
    - Pin AVDD & IOVDD à 3.3V (pas de 5V)
    - Pin LPn à 3.3V
*/
#define IR_SCL_PIN 22 // Ligne SCL de l'I2C du capteur ToF
#define IR_SDA_PIN 21 // Ligne SDA de l'I2C du capteur ToF
#define IR_LPN_PIN 0  // Ligne LPn du capteur ToF (pour le réveiller) - Branché au 3.3V direct c'est plus simple

/*POMPE*/

#define POMPE_PIN 17

/*
SERVO
*/
#define SERVPIN 18 // Broche du servo moteur sur D15
#define ANGLE1 0   // Valeur en degrés comprises entre 0° et 180° max
#define ANGLE2 180
#define TEMPS_BLINK 1000 // Temps clignotement (ms).


// Le coté est défini quand on est face à la scène.
//  equipe = 1 : on est du coté gauche (jaune).
//  equipe = 0 : on est du coté droit (bleue).

#define LED 2             // PIN LED pour le setup
#define PIN_READEQUIPE 19 // PIN pour lire l'interrupteur qui defini l'équipe
#define PIN_TIRETTE 23    // PIN de la tirette pour lancer le match

/*
Paramètres globaux
*/

#define SPEED 200        // Vitesse (en cm/s ?) (255 est la vitesse max des moteurs)
#define DISTANCE_MIN 120 // Distance minimale pour éviter un obstacle en mm
// #define EPSP 100         // Incertitude position, cm
// #define EPSA 0.1         // Incertitude position, radian

/*
Gains naifs pour réellement avancer de 10cm avec un delay
*/
#define K_NAIF 1.402
#define K_ANGLE_NAIF 2.753

/*
Paramètres de l'asservissement
    A régler pour que le robot suive bien sa trajectoire
    Kp : gain proportionnel, plus il est grand plus le robot réagit vite à une erreur de position, mais peut causer des oscillations si trop élevé.
*/
#define KP 1
#define INTERVAL_ASSERV 50 // en ms
#define MARGE_ERREUR_TICKS 50
#define GAIN_CM_TO_TICKS 100.04326 // c'est 20809/208
#define GAIN_ANGLE_TO_TICKS 13.1 // à régler à la main avec un 3-6 no scope.
#define DELAY_TIME 500 //pour le delay entre 2 actions de déplacement


// Define Positions en fonction des équipe (J = JAUNE (gauche), B = BLEU (droite))

#define J_POSITION_DEPART_X 0
#define J_POSITION_DEPART_Y 0

// TODO : changer les positions de départ pour bleu
#define B_POSITION_DEPART_X 0
#define B_POSITION_DEPART_Y 0

#endif