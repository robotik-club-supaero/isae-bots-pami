/**
 * @file main.cpp
 * @brief Programme principal , a implementer dans la pami
 */

#include <Pami.h>
#include <Arduino.h>
#include <ESP32Encoder.h>

// Initialise les différents objets
// Ultrason ultrason = Ultrason(ULTRASON_ECHO, ULTRASON_TRIGGER);
Serv servo = Serv(SERVPIN);
Irsensor ir_sensor = Irsensor(IR_SDA_PIN, IR_SCL_PIN, IR_LPN_PIN);
Moteur moteur_r = Moteur(EN_R, IN1_R, IN2_R, INV_MOT_R);
Moteur moteur_l = Moteur(EN_L, IN1_L, IN2_L, INV_MOT_L);
Encodeur encodeur_r = Encodeur(CLK_R, DT_R, INV_ENC_R);
Encodeur encodeur_l = Encodeur(CLK_L, DT_L, INV_ENC_L);

// La pami en elle même
Pami pami = Pami(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &servo, &ir_sensor);

unsigned long newtime;
unsigned long oldtime;
unsigned long newtime_ir;
unsigned long oldtime_ir;

int equipe;
float start_time;

// initialisation de la strat avec les étapes à 0
// ne VRAIMENT pas le bouger d'ici, notamment pas le mettre dans le setup
// c'est une variable extern donc ici on lui assigne juste sa valeur
int etape_globale = 0;

void delay_non_bloquant(int etape_d_appel, unsigned long oldtime, float delay_time)
{
    if ((millis() - oldtime > delay_time) && (etape_globale == etape_d_appel))
    {
        Serial.println("Etape_global : " + String(etape_globale));
        etape_globale++; // suivant
    }
};

void setup()
{
    Serial.begin(115200); // Initialisation de la communication série
    delay(6000);
    Serial.println("---------- Setup starting ----------");

    // Setup capteur IR
    ir_sensor.setup();
    Serial.println("Setup Done : IR Sensor");

    // Setup servo
    servo.setup();
    Serial.println("Setup Done : Servo");

    // Setup moteur droit & gauche
    moteur_l.setup();
    moteur_r.setup();
    Serial.println("Setup Done : Moteurs");

    // Setup encodeur droit & gauche
    encodeur_r.setup();
    encodeur_l.setup();
    Serial.println("Setup Done : Encodeurs");

    // LED intégrée à l'ESP32 pour blink quand la pompe est activée
    pinMode(PIN_LED, OUTPUT);

    // Pompe
    pinMode(PIN_POMPE, OUTPUT);

    // TIRETTE :
    pinMode(PIN_TIRETTE, INPUT);
    bool etat_tirette = digitalRead(PIN_TIRETTE);
    Serial.print(etat_tirette == 1 ? "Tirette en place \t" : "Tirette enlevée \t");

    // Interrupteur choix équipe
    pinMode(PIN_READEQUIPE, INPUT);
    equipe = digitalRead(PIN_READEQUIPE);
    String color_equipe = (equipe == 1) ? "JAUNE" : "BLEUE";
    Serial.print("Equipe : ");
    Serial.println(color_equipe);

    Serial.println("---------- Setup over ----------\n\n");

    // On attends le début du match, on mettra ensuite toute la stratégie dans la loop qui tourne en continu
    bool tirette_en_place = digitalRead(PIN_TIRETTE);
    while (tirette_en_place)
    {
        delay(10);
        tirette_en_place = digitalRead(PIN_TIRETTE);
    }
    Serial.println("Tirette enlevée, début du match");

    // On remet a 0 les positions car la roue tourne pendant l'upload
    // (car l'esp32 utilise le pin du moteur pendant l'upload)
    encodeur_l.clear_count();
    encodeur_r.clear_count();

    pami.tourner_asservi(0, 360, oldtime);

    // On commence les timers
    oldtime = millis();
    newtime = millis();

    start_time = millis();

    oldtime_ir = millis();
    newtime_ir = millis();
}

void loop()
{
    // Pour faire des delais :
    // penser à ne pas updater le temps dans les délais !
    // remplir ici :
    // int etapes_avec_delays[] = {1,3,5};
    // int taille_tab = sizeof(etapes_avec_delays)/sizeof(int);

    // for (int i = 0; i < taille_tab; i++) {
    //     if (etape_globale != etapes_avec_delays[i]) {
    //         oldtime = newtime;
    //         break;
    //     }
    // }
    oldtime = newtime;

    if (millis() - oldtime > 8 * INTERVAL_ASSERV)
    {
        Serial.println("Etape globale = " + String(etape_globale) + "\t");
        // Serial.println("\t Distance IR = " + String(ir_sensor.ir_minimum_distance));
    }

    // Si on veut de l'IR :
    // oldtime_ir = newtime_ir;
    // newtime_ir = ir_sensor.loop(oldtime_ir);
    // if (ir_sensor.ir_minimum_distance<DISTANCE_MIN && ir_sensor.ir_minimum_distance>5){
    //     moteur_l.set_speed(0);
    //     moteur_r.set_speed(0);
    // }
    // else { // mettre ici la strat }

    float angle = (equipe == 1) ? 90 : -90;
    float angleFinal = 25;
    float angleLourd = (equipe == 1) ? 100 : -100;

    if (equipe == 0)
    {
        // tomber 1er frigo
        newtime = pami.avancer_asservi(0, 43, oldtime);
        newtime = pami.tourner_asservi(1, -angle, oldtime);
        newtime = pami.avancer_asservi(2, 26, oldtime);

        // recallage 1
        newtime = pami.avancer_asservi(3, -33.5, oldtime);
        newtime = pami.avancer_asservi(4, 6, oldtime);

        // avance vers 2ème caisse
        newtime = pami.tourner_asservi(5, angle, oldtime);
        newtime = pami.avancer_asservi(6, 27, oldtime);
        newtime = pami.tourner_asservi(7, -angle, oldtime);

        // tomber 2eme frigo
        newtime = pami.avancer_asservi(8, 23, oldtime);

        // recallage 2
        newtime = pami.avancer_asservi(9, -31, oldtime);
        newtime = pami.avancer_asservi(10, 6, oldtime);

        // aller vers garde manger
        newtime = pami.avancer_asservi(11, 17, oldtime);
        newtime = pami.tourner_asservi(12, -angle, oldtime);
        newtime = pami.avancer_asservi(13, 28, oldtime);

        // Recuperer pourri 1
        newtime = pami.bouger_servo_non_bloquant(14, 0, oldtime);
        newtime = pami.allumer_pompe(15, oldtime);
        newtime = pami.avancer_asservi(16, -23, oldtime);

        // lacher pourri 1
        newtime = pami.eteindre_pompe(17, oldtime);
        newtime = pami.bouger_servo_non_bloquant(18, 45, oldtime);

        // recallage 3
        newtime = pami.tourner_asservi(19, angle, oldtime);
        newtime = pami.avancer_asservi(20, -30.5, oldtime);
        newtime = pami.avancer_asservi(21, 5, oldtime);

        // aller recuperer pourri 2
        newtime = pami.tourner_asservi(22, -angle, oldtime);
        newtime = pami.avancer_asservi(23, 48, oldtime);
        newtime = pami.tourner_asservi(24, angle, oldtime);

        // recuperer pourri 2
        newtime = pami.bouger_servo_non_bloquant(25, 0, oldtime);
        newtime = pami.allumer_pompe(26, oldtime);

        // aller à lacher pourri 2
        newtime = pami.avancer_asservi(27, -4, oldtime);
        newtime = pami.tourner_asservi(28, angle, oldtime);
        newtime = pami.avancer_asservi(29, 43, oldtime);
        newtime = pami.tourner_asservi(30, -angle / 2, oldtime);
        newtime = pami.eteindre_pompe(31, oldtime);

        // recallage 4
        newtime = pami.tourner_asservi(32, -angle / 2, oldtime);
        newtime = pami.avancer_asservi(33, -25, oldtime);

        // avancer vers bord de scene
        newtime = pami.avancer_asservi(34, 30, oldtime);

        // angle pour être sur la zone
        newtime = pami.tourner_asservi(35, angleFinal, oldtime);
    }
    else
    {
        // tomber 1er frigo
        newtime = pami.avancer_asservi(0, 43, oldtime);
        newtime = pami.tourner_asservi(1, -angle, oldtime);
        newtime = pami.avancer_asservi(2, 26, oldtime);

        // recallage 1
        newtime = pami.avancer_asservi(3, -33.5, oldtime);
        newtime = pami.avancer_asservi(4, 6, oldtime);

        // avance vers 2ème caisse
        newtime = pami.tourner_asservi(5, angle, oldtime);
        newtime = pami.avancer_asservi(6, 27, oldtime);
        newtime = pami.tourner_asservi(7, -angle, oldtime);

        // tomber 2eme frigo
        newtime = pami.avancer_asservi(8, 23, oldtime);

        // recallage 2
        newtime = pami.avancer_asservi(9, -31, oldtime);
        newtime = pami.avancer_asservi(10, 6, oldtime);
 
        // aller vers garde manger
        newtime = pami.avancer_asservi(11, 16, oldtime);
        newtime = pami.tourner_asservi(12, -angle, oldtime);
        newtime = pami.avancer_asservi(13, 30, oldtime);

        // Recuperer pourri 1
        newtime = pami.bouger_servo_non_bloquant(14, 0, oldtime);
        newtime = pami.allumer_pompe(15, oldtime);
        newtime = pami.avancer_asservi(16, -23, oldtime);

        // lacher pourri 1
        newtime = pami.eteindre_pompe(17, oldtime);
        newtime = pami.bouger_servo_non_bloquant(18, 45, oldtime);

        // recallage 3
        newtime = pami.tourner_asservi(19, angle, oldtime);
        newtime = pami.avancer_asservi(20, -30.5, oldtime);
        newtime = pami.avancer_asservi(21, 5, oldtime);

        // aller recuperer pourri 2
        newtime = pami.tourner_asservi(22, -angle, oldtime);
        newtime = pami.avancer_asservi(23, 43, oldtime);
        newtime = pami.tourner_asservi(24, angle, oldtime);

        // recuperer pourri 2
        newtime = pami.bouger_servo_non_bloquant(25, 0, oldtime);
        newtime = pami.allumer_pompe(26, oldtime);

        // aller à lacher pourri 2
        newtime = pami.avancer_asservi(27, -4, oldtime);
        newtime = pami.tourner_asservi(28, angle, oldtime);
        newtime = pami.avancer_asservi(29, 46, oldtime);
        newtime = pami.tourner_asservi(30, -angle / 2, oldtime);
        newtime = pami.eteindre_pompe(31, oldtime);

        // recallage 4
        newtime = pami.tourner_asservi(32, -angle / 2, oldtime);
        newtime = pami.avancer_asservi(33, -9.5, oldtime);

        // avancer vers bord de scene
        newtime = pami.avancer_asservi(34, 30, oldtime);

        // angle pour être sur la zone
        newtime = pami.tourner_asservi(35, angleFinal, oldtime);
    }

    // --- SÉQUENCE DE 100 ÉTAPES DE SERVO (ÉTAPES 36 À 136)--
    // --- SÉQUENCE DE MOUVEMENTS DE SERVO (ÉTAPES 36 À 250) ---
    newtime = pami.bouger_servo_non_bloquant(36, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(37, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(38, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(39, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(40, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(41, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(42, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(43, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(44, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(45, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(46, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(47, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(48, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(49, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(50, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(51, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(52, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(53, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(54, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(55, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(56, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(57, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(58, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(59, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(60, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(61, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(62, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(63, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(64, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(65, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(66, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(67, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(68, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(69, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(70, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(71, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(72, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(73, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(74, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(75, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(76, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(77, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(78, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(79, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(80, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(81, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(82, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(83, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(84, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(85, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(86, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(87, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(88, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(89, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(90, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(91, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(92, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(93, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(94, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(95, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(96, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(97, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(98, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(99, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(100, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(101, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(102, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(103, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(104, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(105, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(106, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(107, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(108, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(109, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(110, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(111, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(112, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(113, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(114, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(115, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(116, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(117, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(118, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(119, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(120, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(121, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(122, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(123, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(124, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(125, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(126, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(127, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(128, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(129, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(130, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(131, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(132, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(133, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(134, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(135, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(136, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(137, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(138, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(139, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(140, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(141, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(142, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(143, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(144, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(145, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(146, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(147, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(148, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(149, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(150, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(151, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(152, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(153, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(154, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(155, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(156, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(157, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(158, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(159, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(160, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(161, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(162, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(163, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(164, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(165, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(166, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(167, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(168, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(169, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(170, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(171, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(172, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(173, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(174, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(175, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(176, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(177, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(178, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(179, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(180, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(181, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(182, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(183, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(184, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(185, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(186, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(187, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(188, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(189, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(190, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(191, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(192, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(193, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(194, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(195, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(196, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(197, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(198, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(199, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(200, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(201, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(202, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(203, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(204, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(205, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(206, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(207, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(208, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(209, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(210, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(211, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(212, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(213, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(214, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(215, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(216, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(217, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(218, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(219, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(220, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(221, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(222, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(223, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(224, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(225, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(226, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(227, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(228, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(229, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(230, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(231, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(232, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(233, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(234, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(235, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(236, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(237, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(238, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(239, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(240, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(241, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(242, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(243, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(244, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(245, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(246, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(247, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(248, 0, oldtime);
    newtime = pami.bouger_servo_non_bloquant(249, 90, oldtime);
    newtime = pami.bouger_servo_non_bloquant(250, 0, oldtime);
}