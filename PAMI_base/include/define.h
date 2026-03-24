/**
 * @file define.h
 * @brief fichier de configuration des pins pour la pami )
 * Voir avec l'elec pour les pins
 */

// TODO REGLER LES PINS AVANT LA COUPE AVEC L'ELEC
#ifndef DEFINE_H
#define DEFINE_H

/*
ENCODEURS
*/
#define CLK_R 4 // CLK RIGHT ENCODER
#define DT_R 16 // DT RIGHT ENCODER

#define CLK_L 19 // CLK LEFT ENCODER
#define DT_L 18  // DT LEFT ENCODER

#define INV_R 0 // Inversion du sens de rotation de l'encodeur droit
#define INV_L 1 // Inversion du sens de rotation de l'encodeur gauche

/*
MOTEURS
*/
#define EN_R 13  // M1 EN RIGHT MOTEUR
#define IN1_R 12 // M1 IN1 RIGHT MOTEUR
#define IN2_R 14 // M1 IN2 RIGHT MOTEUR

#define EN_L 25  // M2 EN LEFT MOTEUR
#define IN1_L 26 // M2 IN1 LEFT MOTEUR
#define IN2_L 27 // M2 IN2 LEFT MOTEUR

/*
SENSORS (IR, Ultrason, ect...)
L'IR Sensor demande :
    - PIN I2C SCL & SDA définits dessous
    - Pin AVDD & IOVDD à 3.3V (pas de 5V)
    - Pin LPn à 3.3V
*/
#define ULTRASON_ECHO 0    // Ultrason echo pin
#define ULTRASON_TRIGGER 0 // Ultrason trigger pin

#define IR_SCL_PIN 5  // Ligne SCL de l'I2C du capteur ToF
#define IR_SDA_PIN 17 // Ligne SDA de l'I2C du capteur ToF
#define IR_LPN_PIN 2  // Ligne LPn du capteur ToF (pour le réveiller)

/*
SERVO
*/
#define SERVPIN 15 // Broche du servo moteur sur D15
#define ANGLE1 0   // Valeur en degrés comprises entre 0° et 180° max
#define ANGLE2 180
#define TEMPS_BLINK 1000 // Temps clignotement (ms).

/*
MACHINE A ETAT
*/
// Le coté est défini quand on est face à la scène.
//  equipe = 1 : on est du coté gauche (jaune).
//  equipe = 0 : on est du coté droit (bleue).

#define LED 2             // PIN LED pour le setup
#define PIN_READEQUIPE 22 // PIN pour lire l'interrupteur qui defini l'équipe
#define PIN_TIRETTE 21    // PIN de la tirette pour lancer le match

/*
    Low & Low : Première (collée au mur)
    Low & High : Deuxième
    High & Low : Troisième
    High & High : Quatrième
*/
#define PIN_INT_PAMI_1 110 // PIN interrupteur 1 pour le numéro de la PAMI
#define PIN_INT_PAMI_2 110 // PIN interrupteur 2 pour le numéro de la PAMI

/*
Paramètres globaux
*/
#define GLOBALTIME 99000 // Temps global de la pami en ms (99s)
#define START_TIME 2000  // Les pamis commencent dans les 15 dernières secondes.
#define SPEED 150        // Vitesse (en cm/s ?) (255 est la vitesse max des moteurs)
#define DISTANCE_MIN 100 // Distance minimale pour éviter un obstacle en mm
#define EPSP 1           // Incertitude position, cm
#define EPSA 0.1         // Incertitude position, radian

// Define Positions en fonction des équipe (J = JAUNE (gauche), B = BLUE (droite))
// Chaque pami à ses propres positions

// PAMI 1 (collée au mur)
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

#define J_POSITION_3_FINAL_X 0
#define J_POSITION_3_FINAL_Y 0

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