#include "global.h"
#ifndef CAPTEURS_H
#define CAPTEURS_H

#include <Arduino.h>
//#include "wiring_analog.h"
//#include "wiring_digital.h"
//#include "wiring_constants.h"

/**********************************************
Ce fichier décrit les variables et fonctions 
relatives à la gestion des capteurs d'APEROBOT
***********************************************/

//--- TYPES de capteurs analogiques
#define GP2Y0E03      0
#define GP2Y0A21YK0F  1

//--- CODES  des capteurs
#define captAnaD      0
#define captAnaG      1
#define captGG        2
#define captGC        3
#define captDC        4
#define captDD        5
#define nbCaptAna     6
#define dernierCapt   5

//--- CABLAGES DES CAPTEURS
#define ANAPD       A0
#define ANAPG       A1 
#define ANADD       A2
#define ANADC       A3 
#define ANAGC       A4 
#define ANAGG       A5 

//--- Aspect Mesure Distance des GP2Y0E03
/* Les coefficients y et b sont calculés à partir
 * de la droite caractéristique donnée par 
 * le constructeur à partir des deux points 
 * 10cm = a.2V+b et 50cm = a.0,5V+b
 * on en trouve a = -0.036 V/cm et b = 2.036 V
 * on en déduit D = 1/a * U - (b/a) 
 * noté ici dist = penteCapt * CAN * quantum - offsetCapt
 * q est le quantum du convertisseur = 5V/1024 sur UNO/Nano
*/
#define penteCapt     -25.8       // unité mm/V
#define offsetCapt    61.61       // unité mm
//#define quantum       0.00488    // 5V/unité de CAN sur UNO/Nano/
#define quantum       0.00322     // XIAO Nucléo32 10 bits 3.3/1024 
//#define quantum       3.3/4096   // unité de CAN sur STM32F411

//--- Aspect Mesure Distance des GP2Y0A21YK0F
/* Les coefficients y et b sont calculés à partir
 * de la caractéristique supposée droite donnée par V(V)*D(cm) = 2.7 (V.dm
 * le plus utile est entre 40 et 20cm pour lesquelles on trouve  
 * 4dm*0.75V = a et 3dm*0.9V = a On trouve a environ égal à 2.8V.dm
 * à partir de V on trouve D = (2.8/V)dm.
 * On tiendra compte d'une correction en D = (a + 2.8) / V + c
 * où a est le correcteur de coefficient inversion et c le correcteur d'offset
 * noté ici dist = y * CAN * q - c
 * q est le quantum du convertisseur = 5V/1024 sur UNO/Nano
*/

struct CaptAna          // données associées à ces capteurs
{
  byte   code;          // numéro de repérage
  byte   type;          // GP2Y0E03 ou GP2Y0A21YK0F
  String libelle;       // nom du rôle
  uint16_t port;        // nom du port employé
  uint16_t lecture;     // distance lue après correction en mm
  double coeffDist;     // coefficient correcteur de distance
  double offsetDist;    // correction d'offset
  double coeffInversion;  // pour les 4 capteurs IR arrière, correcteur du terme d'inversion 
};
CaptAna captAnas[nbCaptAna];

//--- Aspect Rangement des Distances des 4 capteurs obstacles
#define DDroite 3
#define CDroite 2
#define CGauche 1
#define GGauche 0
double Distances[4];   // tableau des mesures courantes

//*** ASPECT TEMPOREL DU CAPTEUR OTOS
uint16_t rythmeOtos;

/**********     FONCTIONS DES CAPTEURS LOGIQUES  ****************/

void initCapteursLogiques()
{         // cette fonction initialise les voies des capteurs logiques
  pinMode (LANCEUR, INPUT_PULLUP);
  pinMode (COTE, INPUT_PULLUP);
  pinMode (PLANCHE, INPUT_PULLUP);
  pinMode (ASCBAS, INPUT_PULLUP);
  pinMode (COTEBANDEG, INPUT_PULLUP);
  pinMode (COTEBANDED, INPUT_PULLUP);
}

bool litLanceur()
{
  return (digitalRead(LANCEUR) == LOW);
}

bool litCote()
{
  return (digitalRead(COTE) == LOW);
}

bool litAscBas()
{
  return (digitalRead(ASCBAS) == LOW);
}

bool litPlanche()
{
  return (digitalRead(PLANCHE) == LOW);
}

bool litCoteBandeD()
{
  return (digitalRead(COTEBANDED) == LOW);
}

bool litCoteBandeG()
{
  return (digitalRead(COTEBANDEG) == LOW);
}

byte construitCaptLogiques()
{         // caette fonction compose l'octet état des capteurs logique
  byte bb = 0;
  if (litLanceur())       bb += 0x80;
  if (litCote())          bb += 0x40;
  if (litAscBas())        bb += 0x20;
  if (litCoteBandeG())    bb += 0x4;
  if (litCoteBandeD())    bb += 0x2;
  if (litPlanche())       bb += 0x1;
  return (bb); 
}

/**********     FONCTIONS DES CAPTEURS ANALOGIQUES  ****************/

void creeCapteurAnalogique()
{       // cette fonction construit le tableau des données des capteurs analogiques
  captAnas[captAnaD].code     = captAnaD;
  captAnas[captAnaD].type     = GP2Y0E03;
  captAnas[captAnaD].libelle  = "Capteur fin de planche a Droite";
  captAnas[captAnaD].port     = ANAPD;
  captAnas[captAnaD].coeffDist = 1.0;
  captAnas[captAnaD].offsetDist = 0.0;

  captAnas[captAnaG].code = captAnaG;
  captAnas[captAnaG].type = GP2Y0E03;
  captAnas[captAnaG].libelle = "Capteur fin de planche a Gauche";
  captAnas[captAnaG].port     = ANAPG;
  captAnas[captAnaG].coeffDist = 1.0;
  captAnas[captAnaG].offsetDist = 0.0;

  captAnas[captGG].code = captGG;
  captAnas[captGG].type = GP2Y0A21YK0F;
  captAnas[captGG].libelle = "Capteur obstacle à gauche toute";
  captAnas[captGG].port     = ANAGG;
  captAnas[captGG].coeffDist = 1.0;
  captAnas[captGG].offsetDist = 0.0;
  captAnas[captGG].coeffInversion = 0.0;

  captAnas[captGC].code = captGC;
  captAnas[captGC].type = GP2Y0A21YK0F;
  captAnas[captGC].libelle = "Capteur obstacle à gauche central";
  captAnas[captGC].port     = ANAGC;
  captAnas[captGC].coeffDist = 1.0;
  captAnas[captGC].offsetDist = 0.0;
  captAnas[captGC].coeffInversion = 0.0;

  captAnas[captDC].code = captDC;
  captAnas[captDC].type = GP2Y0A21YK0F;
  captAnas[captDC].libelle = "Capteur obstacle à droite central";
  captAnas[captDC].port     = ANADC;
  captAnas[captDC].coeffDist = 1.0;
  captAnas[captDC].offsetDist = 0.0;
  captAnas[captDC].coeffInversion = 0.0;

  captAnas[captDD].code = captDD;
  captAnas[captDD].type = GP2Y0A21YK0F;
  captAnas[captDD].libelle = "Capteur obstacle à droite toute";
  captAnas[captDD].port     = ANADD;
  captAnas[captDD].coeffDist = 1.0;
  captAnas[captDD].offsetDist = 0.0;
  captAnas[captDD].coeffInversion = 0.0;
}

uint16_t litAnaCaptD()
{       // cette fonction fournit le nombre image de la distance du capteur fin de planche Droite
  uint16_t val = 0;
  double dist;
  for (byte lect = 0; lect < 4; lect++)
      val += analogRead(captAnas[ANAPD].port);    // cumul des 4 mesures
  val = val >> 2; // moyennage par division par 4
  if (DEBUGMES)
    Serial.printf ("Mesure = %d à Droite des planches", val);
  // calcule de la distance  : emploi de la courbe V = y.d+b
  dist =  penteCapt * val * quantum * captAnas[captAnaD].coeffDist + offsetCapt + captAnas[captAnaD].offsetDist;
  captAnas[captAnaD].lecture = int(dist * 10);   // pour avoir une mesure en mm
  if (DEBUGMES)
    if (dist < 60.0)     // si plus de 60cm, le résultat n'est plus assez fiable
    {
      Serial.print ("  | Distance = ");
      Serial.print (captAnas[captAnaD].lecture);
      Serial.println (" mm");
    }
    else 
      Serial.println ("   | Trop loin !");
    return(int(dist));
}

uint16_t litAnaCaptG()
{       // cette fonction fournit le nombre image de la distance du capteur fin de planche Gauche
  uint16_t val = 0;
  double dist;
  for (byte lect = 0; lect < 4; lect++)
      val += analogRead(captAnas[ANAPG].port);    // cumul des 4 mesures
  val = val >> 2; // moyennage par division par 4
  if (DEBUGMES)
    Serial.printf ("Mesure = %d à Gauche des planches", val);
  // calcule de la distance  : emploi de la courbe V = y.d+b
  dist =  penteCapt * val * quantum * captAnas[captAnaG].coeffDist + offsetCapt + captAnas[captAnaG].offsetDist;
  captAnas[captAnaG].lecture = int(dist * 10);    // pour avoir une mesure en mm
  if (DEBUGMES)
    if (dist < 60.0)     // si plus de 60cm, le résultat n'est plus assez fiable
    {
      Serial.print ("  | Distance = ");
      Serial.print (captAnas[captAnaG].lecture);
      Serial.println (" mm");
    }
    else 
      Serial.println ("   | Trop loin !");
    return(int(dist));
}

void lit4CaptObstacle()
{     // lit 4 fois les 4 capteurs obstacle, fait les moyennes et range les distances dans le tableau
  uint16_t val[4];          // tableau où l'on range l'accumulation des mesures des 4 capteurs
  for (byte lect = 0; lect < 4; lect++)     // comptage des lectures de la campagne
  {
    val[0] = val[1] = val[2] = val[3] = 0;  // on initialise ce qui va s'accumuler
    for (byte capt = captGG; capt <= dernierCapt; capt++)       // on explore les 4 capteurs 
      val[capt - captGG] += analogRead(captAnas[capt].port);    // cumul des 4 mesures
  }
  for (byte lect = 0; lect < 4; lect++)
    val[lect] = val[lect] >> 2; // moyennage par division par 4 des mesures de chaque capteur
  // selon le type, on calcule la distance
  double dist;
  for (byte capt = captGG; capt <= captDD; capt++)
  {
    if (captAnas[capt].type == GP2Y0E03)
    {
      dist =  penteCapt * val[capt-captGG] * quantum * captAnas[capt].coeffDist + offsetCapt + captAnas[capt].offsetDist; // emploi de la courbe V = y.d+b
      captAnas[capt].lecture = int(dist);      // pour avoir une mesure en centimètres
    }
    if (captAnas[capt].type == GP2Y0A21YK0F)
    {
      dist = (captAnas[capt].coeffInversion + 2.8) / (quantum * val[capt-captGG]) + captAnas[capt].offsetDist;
      captAnas[capt].lecture = int(dist * 10);      // pour avoir une mesure en centimètres à partir des dm
    }
    if (DEBUGMES)
    {
      for (byte capt = captGG; capt <= captDD; capt++)
      {
        if (captAnas[capt].lecture < 60)     // si plus de 60cm, le résultat n'est plus assez fiable
          Serial.printf (" Pour %d Distance = % d cm | ", captAnas[capt].code, captAnas[capt].lecture);
        else 
          Serial.println ("   Trop loin !   |");
      }
    }
  }
}


#endif