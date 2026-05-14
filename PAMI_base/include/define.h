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
#define EN_L 32  // M1 EN LEFT MOTEUR
#define IN1_L 14 // M1 IN1 LEFT MOTEUR
#define IN2_L 27 // M1 IN2 LEFT MOTEUR

#define EN_R 26  // M2 EN RIGHT MOTEUR
#define IN1_R 25 // M2 IN1 RIGHT MOTEUR
#define IN2_R 33 // M2 IN2 RIGHT MOTEUR

/*
ENCODEURS
*/
#define CLK_L 39 // C1 LEFT ENCODER
#define DT_L 36  // C2 LEFT ENCODER

#define CLK_R 35 // C1 RIGHT ENCODER
#define DT_R 34  // C2 RIGHT ENCODER

/*
A tester avec print_vitesse ou print_encodeur pour que les vitesses soient positives et que le robot avance
    Mais modifier le sens moteur change le sens encodeur
    Mais modifier le sens encodeur ne change pas le sens moteur
# Mention abominable
*/
#define INV_ENC_R 1 // Inversion du sens de rotation de l'encodeur droit
#define INV_MOT_R 0 // Inversion du sens de rotation du moteur droit

#define INV_ENC_L 0 // Inversion du sens de rotation de l'encodeur gauche
#define INV_MOT_L 1 // Inversion du sens de rotation du moteur gauche

/*
SENSORS (IR, Ultrason, ect...)
L'IR Sensor demande :
    - PIN I2C SCL & SDA définits dessous
    - Pin AVDD & IOVDD à 3.3V (pas de 5V)
    - Pin LPn à 3.3V
*/
#define ULTRASON_ECHO 0    // Ultrason echo pin
#define ULTRASON_TRIGGER 0 // Ultrason trigger pin

#define IR_SCL_PIN 22 // Ligne SCL de l'I2C du capteur ToF
#define IR_SDA_PIN 21 // Ligne SDA de l'I2C du capteur ToF
#define IR_LPN_PIN 0  // Ligne LPn du capteur ToF (pour le réveiller) - Branché au 5V direct c'est plus simple

/*
SERVO
*/
#define SERVPIN 18 // Broche du servo moteur sur D15
#define ANGLE1 0   // Valeur en degrés comprises entre 0° et 180° max
#define ANGLE2 180
#define TEMPS_BLINK 500 // Temps clignotement (ms).

/*
MACHINE A ETAT
*/
// Le coté est défini quand on est face à la scène.
//  equipe = 1 : on est du coté gauche (jaune).
//  equipe = 0 : on est du coté droit (bleue).

#define PIN_LED 2         // PIN LED pour le setup
#define PIN_READEQUIPE 19 // PIN pour lire l'interrupteur qui defini l'équipe
#define PIN_TIRETTE 23    // PIN de la tirette pour lancer le match

/*
    Low & Low : Première (collée au mur)
    Low & High : Deuxième
    High & Low : Troisième
    High & High : Quatrième
*/
#define PIN_INT_PAMI_1 16 // PIN interrupteur 1 pour le numéro de la PAMI
#define PIN_INT_PAMI_2 17 // PIN interrupteur 2 pour le numéro de la PAMI

/*
Paramètres globaux
*/
#define GLOBALTIME 29000 // Temps global de la pami en ms (99s)
#define START_TIME 5000  // Les pamis commencent dans les 15 dernières secondes.
#define SPEED 255        // Vitesse (en cm/s ?) (255 est la vitesse max des moteurs)
#define DISTANCE_MIN 100 // Distance minimale pour éviter un obstacle en mm
#define EPSP 100         // Incertitude position, cm
#define EPSA 0.1         // Incertitude position, radian

/*
Gains naifs pour réellement avancer de 10cm avec un delay
*/
#define K_NAIF 1.402
#define K_ANGLE_NAIF 1.7

/*
Paramètres de l'asservissement
    A régler pour que le robot suive bien sa trajectoire
    Kp : gain proportionnel, plus il est grand plus le robot réagit vite à une erreur de position, mais peut causer des oscillations si trop élevé.
    Ti : gain intégral, permet de corriger les erreurs persistantes en accumulant les erreurs passées, mais peut causer des oscillations si trop élevé.
    Td : gain dérivé, permet de réduire les oscillations en anticipant les erreurs futures, mais peut rendre le système instable si trop élevé.
*/
#define KP_r 0.085
#define KP_l 0.090
#define KP_angle 0.0083

#define TI_r 100
#define TI_l 100
#define TI_angle 0

#define TD_r 0
#define TD_l 0
#define TD_angle 0

// Define Positions en fonction des équipe (J = JAUNE (gauche), B = BLUE (droite))
// Chaque pami à ses propres positions

// PAMI 1 (collée au mur - ninja)
#define J_POSITION_1_DEPART_X 0
#define J_POSITION_1_DEPART_Y 0

#define J_POSITION_1_FINAL_X 0
#define J_POSITION_1_FINAL_Y 0

#define B_POSITION_1_DEPART_X 0
#define B_POSITION_1_DEPART_Y 0

#define B_POSITION_1_FINAL_X 0
#define B_POSITION_1_FINAL_Y 0

// PAMI 2
#define J_POSITION_2_DEPART_X 0
#define J_POSITION_2_DEPART_Y 0

#define J_POSITION_2_FINAL_X 0
#define J_POSITION_2_FINAL_Y 0

#define B_POSITION_2_DEPART_X 0
#define B_POSITION_2_DEPART_Y 0

#define B_POSITION_2_FINAL_X 0
#define B_POSITION_2_FINAL_Y 0

// PAMI 3
#define J_POSITION_3_DEPART_X 0
#define J_POSITION_3_DEPART_Y 0

#define J_POSITION_3_FINAL_X 940.0
#define J_POSITION_3_FINAL_Y -450.0

#define B_POSITION_3_DEPART_X 0
#define B_POSITION_3_DEPART_Y 0

#define B_POSITION_3_FINAL_X 0
#define B_POSITION_3_FINAL_Y 0

// PAMI 4
#define J_POSITION_4_DEPART_X 0
#define J_POSITION_4_DEPART_Y 0

#define J_POSITION_4_FINAL_X 0
#define J_POSITION_4_FINAL_Y 0

#define B_POSITION_4_DEPART_X 0
#define B_POSITION_4_DEPART_Y 0

#define B_POSITION_4_FINAL_X 0
#define B_POSITION_4_FINAL_Y 0

#endif