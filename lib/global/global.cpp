#include <sys/_stdint.h>
/*********************************************************
* Fichier : global.cpp                                  *
* Description : Définition des variables globales       *
* partagées par les différents modules du programme     *
**********************************************************/

#include "global.h"

#if defined(NUCLEO64_BUSLX16)
HardwareSerial BusLX16(PC_5, PC_4);
#elif defined(NUCLEO64_LS126)           // cas de APEROBOT
HardwareSerial BusLX16(PC_5, PC_4);    // (Rxpin, Txpin) du bus servomoteurs
#elif defined(NUCLEO32_LS126)
HardwareSerial BusLX16(D0, D1);
#elif defined(NUCLEO32_BUSLX16) 
HardwareSerial BusLX16(D0, D1);   // voie série servomoteurs LX16 sur Nucléo 32 
#endif

#if defined(NUCLEO64_BUSRS485)    // bus série des steppers
HardwareSerial BusPAP(D2, D8);    // (Rxpin, Txpin)
#endif

// Pointeurs de timers définis selon les #define STM des cartes
#if defined(TIM1)
TIM_TypeDef *OtosTim = TIM1;    // pour rythmer les mesures positions/déplacement
#endif
#if defined(TIM6)
TIM_TypeDef *Tick = TIM6;       // pour rythmer la mesure des distances avec les capteurs avant
#endif

HardwareTimer *MyTimOtos = new HardwareTimer(OtosTim);
HardwareTimer *MyTimM    = new HardwareTimer(Tick);

// Variables globales
const uint8_t acc_globale = 50;
int vitesse_nom = 100;
bool deg_rad = RADIAN;

Mvt mvts[10] = {};

unsigned long heure = 0;
bool parle = false;
bool rep485_en_cours = false;

uint8_t mot_arriere = 1;
uint8_t mot_droit   = 2;
uint8_t mot_gauche  = 3;
uint8_t ascenseur   = 4;
int8_t mot_tous     = -1;


