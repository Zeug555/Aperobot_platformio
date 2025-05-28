/*********************************************************
* Fichier : global.h                                    *
* Description : Déclarations globales et constantes     *
* mécaniques, temps, liaison série, broches et moteurs  *
* Utilisé dans tout le projet                           *
**********************************************************/

#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <HardwareSerial.h>

// === DRAPEAUX DEBUG (affichage série)
#define AFFICHAGE     true
#define DEBUG         true  // général pour les affichages
#define DEBUG_STATUS  false
#define DEBUG_LDP     false
#define DEBUG_AFF_REP false
#define DEBUG_AFF_TR  true
#define DEBUG_AFF_CRC false
#define DEBUG_VIT     false
#define DEBUG_MVT     false
#define DEBUG_PULSE   true     // affichage messages de trames en mvt en déplacement
#define DEBUGUSB      true     // affiche les messages relatifs aux échanges avec RPI4
#define DEBUGCRC      true     // affiche les CRC calculés et comparaisons
#define DEBUGMES      true     // affiche les infos sur les messages échangés
#define DEBUGSEQ      true     // affiche les infos sur la séquence en cours
#define DEBUGSM       true     // affiche les messages relatifs aux servomoteurs
#define DEBUGLITSM    true     // pour les messages reçus émis par les Servomoteurs
#define DEBUGASC      true

//--- Aspect ALIMENTATION
// commenter la ligne qui ne convient pas parmi les deux tensions batterie qui suivent.
#define BATTERIE 20
// #define BATTERIE 15

//*** ASPECT CABLAGE CAPTEURS et COMMUNICATION Révisé le 25 mai 2025
//--- autres actionneurs
#define CDEPOMPE    D5      // sortie PWM de mise en service de la pompe
#define DIO         D9      // signaux d'un éventuel afficheur 4x7segments
#define SCK         D3
//--- Capteurs
#define PLANCHE     PC_15   // fn de cours présence planche en charge
#define COTE        PC_13   // éventuel coté de jeu
#define LANCEUR     PC_14   // lanceur de départ
#define COTEBANDED  PC3     // contact "on est en contact avec le rebord"
#define COTEBANDEG  PC2
#define ASCBAS      PB7   // devenu inutile, maintenant lié au stepper Ascenseur
#define IO1         D10   // signaux de sélections capteur IR en cas d'usage en I2C
#define IO2         D11
//--- capteurs analogiques
#define ANAPD       A0    // capteur bout de planche de droite
#define ANAPG       A1    // capteur bout de planche de gauche
#define ANADD       A2    // Capteur obstacle tout à droite
#define ANADC       A3 
#define ANAGC       A4 
#define ANAGG       A5    // capteur obstacle tout à gauche
//--- Otos I2C
#define A9          D7    // Broches définies dans le câblage de l'OTOX
#define A10         D6   
#define SDAPIN      PB_14   // Signaux I2C pour échanger avec l'OTOX
#define SCLPIN      PB_13

//--- Aspect CARTES INTERFACES
#define NUCLEO64_BUSLX16      // indique que l'on travaille sur L476RG et carte rouge
// #define NUCLEO64_LS126     // indique que l'on travaille sur L476RG et carte verte
// #define NUCLEO32_BUSLX16   // indique que l'on travaille sur L432KC et carte rouge
//#define NUCLEO32_LS126     // indique que l'on travaille sur L432KC et carte verte
#define NUCLEO64_BUSRS485     // indique que l'on emploie les PAP sur L476RG
//***  pour la partie SERVOMOTEURS
//#include <HardwareSerial.h>    //!Doublon
#if defined(NUCLEO64_BUSLX16)      
//HardwareSerial BusLX16(D2, D10);    // voie série servomoteurs LX16 sur Nucléo 64 
extern HardwareSerial BusLX16;
#define SENS2       PB_12
#elif defined(NUCLEO64_LS126)
//HardwareSerial BusLX16(D0, D1);     // voie série servomoteurs LX16 sur Nucléo 32 
extern HardwareSerial BusLX16;
#define SENS        PB_12           // broche employée pour activer le sens TX
#define SENSL       PB_2            // idem pour activer le sens RX
#elif defined(NUCLEO32_LS126)       // pour emploi sur L432KC avec carte verte
extern HardwareSerial BusLX16;    // voie série servomoteurs LX16 sur Nucléo 32
#define SENS        D3              // broche employée pour activer le sens TX
#define SENSL       D4              // idem pour acxtiver le sens RX
#elif defined(NUCLEO32_BUSLX16)     // pour emploi sur L432KC avec carte rouge
extern HardwareSerial BusLX16;
#define SENS2       D3
#endif

//***  pour la partie MOTEURS PAS A PAS
// deux ports série hors USB inenvisageable sur NUCLEO32
#if defined(NUCLEO64_BUSRS485)      
//HardwareSerial BusPAP(PA12, PA11);    // voie série RS485 vers MKS Servo42D 
extern HardwareSerial BusPAP;
#define E_R         D7                // sens du dialogue RS485 
#define PAP         true              // indiquera que les moteurs PAP sont commandés
#else
#define PAP         false             // indiquera que les moteurs PAP sont absents
#endif

#define EMET        true              // émission si signal niveau haut
#define LIT         !EMET

//--- ASPECT TIMERS
// normalement sur L476RG, il y a plusieurs timers disponibles : Tim1, Tim2, ...Tim6 et Tim7
/*
#if defined(TIM1)  
  TIM_TypeDef *OtosTim = TIM1;
#endif
#if defined(TIM2)
  TIM_TypeDef *VoieG = TIM2;
#endif
#if defined(TIM6)
  TIM_TypeDef *Tick = TIM6;
#endif
*/
extern TIM_TypeDef *OtosTim;
// Instantiation des 2 timers. 
// On utilise des pointeurs d'objets (new) 
// pour ne pas les perdre en sortant de la fonction
//HardwareTimer *MyTimOtos = new HardwareTimer(OtosTim); // pour rythmer les mesures positions/déplacement
//HardwareTimer *MyTimM = new HardwareTimer(Tick);    // pour rythmer la mesure des distances avec les capteurs avant
extern HardwareTimer* MyTimOtos;
extern HardwareTimer* MyTimM;

// === CONSTANTES MECANIQUES
#define pas_tour      200
#define micropas      4
#define rayon_embase  118
#define diametre_roue 80
#define tour_roue     (PI * diametre_roue)
#define vitesse_faible 50
#define perimetre_embase (2 * rayon_embase * PI)
#define impulsions_mm ((micropas * pas_tour) / (PI * diametre_roue))
#define impulsions_dg (impulsions_mm * perimetre_embase / 360)
#define DEG_RAD       (360 / (2 * PI))

#define nb_mot    5   // les 3 de l'emabse, l'ascensezur et le broadcast

// === ASPECT MOUVEMENT
#define DEPLACE   true
#define VITESSE   !DEPLACE
#define RADIAN    true
#define DEGRES    !RADIAN

extern const uint8_t acc_globale;
extern int vitesse_nom;
extern bool deg_rad;

// === STRUCTURE DE MOUVEMENT
struct Mvt {
  int vx;
  int vy;
  int vc;
  uint32_t pulses;
  bool type;
};

extern Mvt mvts[10];

// === HORLOGE ET DIALOGUE
extern unsigned long heure;
extern bool parle;
extern bool rep485_en_cours;


// === IDENTIFIANTS MOTEURS
extern uint8_t mot_arriere;
extern uint8_t mot_droit;
extern uint8_t mot_gauche;
extern uint8_t ascenseur;
extern int8_t mot_tous;


#endif