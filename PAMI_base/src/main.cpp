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
Mesure_pos mesure_pos = Mesure_pos(&encodeur_r, &encodeur_l);

// La pami en elle même
Pami pami = Pami(&moteur_r, &moteur_l, &encodeur_r, &encodeur_l, &mesure_pos, &servo, &ir_sensor);

float global_time = 0; // Variable global du temps
float pos_x;
float pos_y;
float angle;

float new_ticks_l, new_ticks_r,newtime;
float old_ticks_l,old_ticks_r,oldtime;

void setup()
{
    Serial.begin(115200); // Initialisation de la communication série
    delay(6000);
    Serial.println("---------- Setup starting ----------");
    
    // Setup capteur IR
    // m_p_ir_sensor->setup();
    // Serial.println("Setup Done : IR Sensor");

    // Setup servo
    // m_p_servo->setup();
    // Serial.println("Setup Done : Servo");

    // Setup mesure position
    mesure_pos.setup();
    Serial.println("Setup Done : Mesure de Position");

    // Setup moteur droit & gauche
    moteur_l.setup();
    moteur_r.setup();
    Serial.println("Setup Done : Moteurs");

    // Setup encodeur droit & gauche
    encodeur_r.setup();
    encodeur_l.setup();
    Serial.println("Setup Done : Encodeurs");


    // LED intégrée à l'ESP32 pour blink quand la configuration est finie
    // pinMode(LED, OUTPUT);
    // digitalWrite(LED, 1); // LED ON pour indiquer le setup réussi


    // TIRETTE :
    pinMode(PIN_TIRETTE, INPUT);
    bool etat_tirette = digitalRead(PIN_TIRETTE);
    Serial.print(etat_tirette==1 ? "Tirette en place \t" : "Tirette enlevée \t");

    // Interrupteur choix équipe
    pinMode(PIN_READEQUIPE, INPUT);
    int read_equipe = digitalRead(PIN_READEQUIPE);
    String color_equipe = (read_equipe == 1) ? "JAUNE" : "BLEUE"; 
    Serial.print("Equipe : ");
    Serial.println(color_equipe);

    // Setup de la position
    if (read_equipe == 1) // JAUNE
    {
        pos_x = J_POSITION_DEPART_X;
        pos_y = J_POSITION_DEPART_Y;
    }
    else
    {
        pos_x = B_POSITION_DEPART_X;
        pos_y = B_POSITION_DEPART_Y;
    }

    Serial.println("---------- Setup over ----------\n\n");

    // On remet a 0 les positions car la roue tourne pendant l'upload 
    // (car l'esp32 utilise le pin du moteur pendant l'upload)
    
    angle=0;

    // On attends le début du match, on mettra ensuite toute la stratégie dans la loop qui tourne en continu
    bool tirette_en_place = digitalRead(PIN_TIRETTE);
    while(tirette_en_place){
        delay(10);
        tirette_en_place = digitalRead(PIN_TIRETTE);
    }
    Serial.println("Tirette enlevée, début du match");
    // On commence le timer
    mesure_pos.reinitialise();
    global_time = millis();



    // pami.distance_target = 0;
    // pami.test(7);

    //setup
}


void loop()
{


    float tick_distance = 1200;
    auto resultat = pami.avancer_asservi(tick_distance,old_ticks_l,old_ticks_r,oldtime);
    
    new_ticks_l=std::get<0>(resultat);
    new_ticks_r=std::get<1>(resultat);
    newtime = std::get<2>(resultat);

    old_ticks_l=new_ticks_l;
    old_ticks_r = new_ticks_r;
    oldtime=newtime;
    
    

    // pami.go_to(100, 0, SPEED);

    // Test avancer ou reculer ou tourner
    // if (i == 0)
    // {
    // pami.avancer(300);
    // pami.print_encodeur();
    // pami.print_position();
    // i = 1;
    // }

    
    // pami.print_log();
    // pami.start_match();

}
