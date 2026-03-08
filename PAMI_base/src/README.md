Code pour les PAMIs -

Régler les gains de l'asservissement
Configurer les pins de chaque composant avec l'élec


Décomposé en plusieurs fichiers (une classe - une fonction)
    - Asservissement    : Asserv & asservPID (le 1ème fichier implémente le correcteur PID du 2ème)
    - Moteur            : Moteur - envoie d'une commande
    - Encodeur          : Encodeur - Récupère la position du robot
    - Mesure Position   : Mesure_pos - Déduit position et vitesse des encodeurs
    - Irsensor          : Capteur ToF (VL53L5CX) - Récupère la distance aux obstacles (tableau de 8 valeurs - moyenne sur chaque colonne)
    - Ultrason          : Capteur ultrason (HC-SR04) - Récupère la distance aux obstacles
    - Machine_etat      : Stratégie du robot
    - Serv              : Servomoteur - Fait bouger un servomoteur entre deux angles
    - main              : A upload sur la pami, assemble toutes les fonctions

Chaque fichier représente une classe qui possède au moins 2 fonctions
    - setup             : Est exécuté au démarrage une seule fois (configuration initiale)
    - loop              : Est exécuté en boucle tant que la pami est allumée
    - Peut possèder d'autres fonctions qui peuvent être appelées dans loop ou setup de cette classe ou d'une autre mais ne sont pas executées automatiquement


Commandes utiles par classes:
    - Asserv :
        - asservissement (vitesse_l_consign, vitesse_r_consign)                 : Permet un asservissement en vitesse
        - asserv_angle (theta_consigne)                                         : Permet un asservissement en angle
        - asserv_global (vitesse_l_consign, vitesse_r_consign, theta_consigne)  : Permet un asservissement en angle et en vitesse
    - Moteur :
        - set_speed (int [0-255]) : Envoie une vitesse au moteur
    - Encodeur :
        - mesure : Retourne le nombre de tics de roue de l'encodeur
    - Mesure Position :
        - loop : Boucle de mise à jour des variables globales de position et vitesse dans le repère des roues et globales
    - Irsensor :
        - loop : Boucle de mise à jour d'une variable vision de la moyenne du capteur sur chacune des 8 colonnes
    - Ultrason :
        - loop : Boucle de mise à jour de la variable m_distance mesurée par le capteur ultrason
    - Machine_etat :
        - setup :
        - loop  :
    - Serv :
        - blink (temps_blink, angle1, angle2) : Fait bouger le servomoteur entre deux angles en un temps donné
    - main :
        - setup : Initialise tous les composants du robot ainsi que l'asservissement
        - loop : Lance la boucle de commande du robot