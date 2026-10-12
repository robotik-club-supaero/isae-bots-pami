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
#define CLK_R 22 // CLK LEFT  ENCODER
#define DT_R 21  // DT LEFT Encoder

#define CLK_L 16 // CLK RIGHT ENCODER
#define DT_L 13  // DT RIGHT Encoder

#define INV_L 0 // Inversion du sens de rotation de l'encodeur gauche
#define INV_R 0 // Inversion du sens de rotation de l'encodeur droit

/*
MOTEURS
*/
#define EN_L 14  // EN RIGHT MOTEUR
#define IN1_L 27 // IN1 RIGHT MOTEUR
#define IN2_L 26 // IN2 RIGHT MOTEUR

#define EN_R 33  // EN LEFT MOTEUR
#define IN1_R 32 // IN1 LEFT MOTEUR
#define IN2_R 25 // IN2 LEFT MOTEUR

/*
SENSORS (IR, Ultrason, ect...)
*/
#define ULTRASON_ECHO 18 // Ultrason echo pin
#define ULTRASON_TRIGGER 19 // Ultrason trigger pin

/*
SERVO
*/
#define SERVPIN 34 // Broche du servo moteur sur D15
#define ANGLE1 20
#define ANGLE2 60
#define TEMPS_BLINK 1 // Temps clignotement (secondes).

/*
MACHINE A ETAT
*/
// Le coté est défini quand on est face à la scène.
//  equipe = 1 : on est du coté gauche (jaune). 
//  equipe = 0 : on est du coté droit (bleue).

#define LED 5 // PIN LED pour le setup
#define PIN_READEQUIPE 17 // PIN pour lire l'interrupteur qui defini l'équipe
#define PIN_TIRETTE 4 // PIN de la tirette pour lancer le match

#define GLOBALTIME 99000 // Temps global de la pami en ms (99s)
#define START_TIME 85000 // Les pamis commencent dans les 15 dernières minutes.
#define SPEED 40         // Vitesse en cm/s (25 ou 255 ? est la vitesse max des moteurs)
#define DISTANCE_MIN 15 // Distance minimale pour éviter un obstacle en cm
#define EPSP 1          // Incertitude position, cm
#define EPSA 0.1        // 0,1 ? //Incertitude position, radian

// Define Positions en fonction des équipe (J = JAUNE (gauche), B = BLUE (droite))
#define J_POSITION_DEPART_X 0 ;
#define J_POSITION_DEPART_Y 0 ;

#define J_POSITION_FINAL_X 0 ;
#define J_POSITION_FINAL_Y 0 ;

#define B_POSITION_DEPART_X 0 ;
#define B_POSITION_DEPART_Y 0 ;

#define B_POSITION_FINAL_X 0 ;
#define B_POSITION_FINAL_Y 0 ;

#endif