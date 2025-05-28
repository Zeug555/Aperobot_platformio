/*********************************************************************
*               Programme APEROBOT_LX16_L476RG_1.INO 
*       par JP Bourguet,       v0.5 du 12 mai 2025
**********************************************************************
* Ce programme fait suite à la version 3 qui pilotait deux grappes de 
* servos à des rythmes indépendants. On se reportera à elle pour 
* comprendre la structure des données et des fonctions.
* Cette version 5 vise à pîloter chaque servomoteur série type LX16A, 
* hors des grappes afin de les mettre dans des postures par des commandes 
* passées depuis un système informatique à travers le port USB de la Nucléo.
**********************************************************************
* On utilise la carte BusLX16-RS485 et un port série RX-Tx d'un
* microcontroleur, ainsi qu'un signal de sens de dialogue pour la mise au point,
* une Nucléo 64 : la NucléoL476RG.
* Cette version 2025 est faite pour piloter le duplexage à travers une carte HC126
**********************************************************************/

#include "Wire.h"
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "monOtos.h"       // fonctions pour capteur OTOS
#include "global.h"        // drapeaux debug / bus séries / cablages
#include "servo42d.h"      // bibliothèque des fonctions de pilotage PAP par RS485
#include "embase3pap.h"
#include "core_debug.h"    // bibliothèque des pilotages des moteurs de l'embase par déplacement
#include "LX16-API.h"      // bibliothèque de fonctions SM unitaires
#include "Actionneurs.h"   // bibliothèque des définitions servos, sequences, pompe
#include "capteurs.h"      // bibliothèque des fonctions tous capteurs sauf OTOS
#include "USB-trames.h"    // bibliothèque des échanges avec USB et les UART

// variables globales
bool Port;                  // port employé pour l'échange en cours de traitement
#define PORTSM  true        // si vrai, ce sont les servomoteurs.
#define PORTPAP !PORTSM     // si faux, ce sont les moteurs pas à pas.

/******************    PROGRAMME PROPREMENT DIT    *********************/

void setup() 
{
  initSerials();            // création des 3 canaux série USB et UART.
  initSens();               // SENS et SENSL sont des sorties de pilotage du halfduplex  
  //--- Aspects capteurs / actionneurs 
  creeActeurs();            // création des données des servos employés
  creeGestes();             // toutes les postures possibles pour tous actionneurs
  creeSequences();          // les deux séquences prévues sont construites
  activeLX16();
  // Otos_init();              // init et calibration du capteur OTOS
  // définition de la fonction d'interruption des Steps appelée 
  //MyTimOtos->attachInterrupt(envoiOtos);
  // mise en attente, avant le lancement du timer
  //MyTimOtos->pause();
  //--- Aspects moteurs PAP (issu du main() de Valentin)
  init_moteurs();
  affectations_moteurs();
  // arret_moteur(-1);         // envoi arret des moteurs par broadcast
  
  ordreRecu = false;
  seqOk = seqPasFinie = false;
  Serial.println (" Init terminée !\n");
}

void loop()
{
  testReceptionOrdre();
  // testReceptionSequence();
  // testAscenceur();
}

void loopFinal() 
{ 
  // boucle de gestion des commandes avec la RPI
  lit_USB();
  if (ordreRecu)
  {
    if (DEBUGUSB)   afficheTrameUtile();  // on affiche les lettres utiles de la trame.
    if (testReception())
      traiteReception();      // exécution des commandes
  }

  // ici on peut simuler les IT au rythmeOtos par millis() 
/*
  if (millis() > (derniereLecture + rythmeOtos))
  {
    Otos.read();        // lancement de la lecture du capteur
    envoiOtos();        // constuction des données et envoi de la trame
  }
*/
  // si une séquence est en cours, c'est ici que l'on rafraichie son état
  if (seqPasFinie)
    derouleSequence(seqOk);     // si seqOk est faux, la séquence est suspendue
}


