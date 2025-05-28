#include "global.h"
#ifndef ACTIONNEURS_H
#define ACTIONNEURS_H

/**************************************************************************
* Cette petite bibliothèque définit les structures, variables et fonctions
* qui permettent à une Nucléo L476RG d'employer un port série UART hardware 
* vers un bus 1 fil pour des servomoteurs de type LX16A et d'autres actionneurs
* On y trouve les définitions et les fonctions des servomoteurs et des grappes.
* En fin, une partie importante relative aux séquences termine la bibliothèque.
****************************************************************************
* A FAIRE :
* - Tester l'initialisation de l'ascenseur (home)
* - Gérer questionnement position / réponse du Servo42D de l'ascenseur
* - 
****************************************************************************/
#include <stdint.h>
//#include "stm32l4xx_ll_rtc.h"
#include <global.h>
#include <LX16-API.h>
#include <servo42d.h>
#include <cstddef>

//*** ASPECTS TEMPORELS
unsigned long debut;        // heure système en ms au début d'une rotation permanente
unsigned long hCde;         // heure de début d'application d'une commande
// delai entre la fin d'une configuration mouvement et le lancement du suivant
uint16_t attenteMvt;   

//*** ASPECT ASCENSEUR
#define pasVis    8                                 // un tour de vis = 8mm
#define pasParMm  ((micropas * pas_tour) / pasVis)  // nombre de pas à faire par mm de mvt ascenseur
#define vitNomAsc 40                                // l'ascenseur tourne à 40 tr/mn

/*****************     STRUCTURES DES ACTIONNEURS   ***************/

//*** ASPECT ACTIONNEURS
struct Acteur
{
  byte      adr;            // numéro identifiant du servo dans le bus 1 .. 253
  byte      grappe;         // dans quelle servos est ce moteur ?  
  byte      posGrappe;      // quelle est sa position temporelle dans la grappe ?
  byte      type;           // type d'actionneur (servo / pompe /pap ...)  
  String    libelle;        // texte décrivant cet organe, son rôle
  bool      mode;           // mode servo:0 / moteur DC:1 pour les LX16
  uint16_t  pos;            // position angulaire 0 .. 1000
  uint32_t  papPos;         // position angulaire pour les pas à pas
  byte      codePos;        // code de la position demandée et positionnée 'X' si en mouvement
  int16_t   vit;            // vitesse de rotation (signée) en moteur (tr/mn pour stepper)
  uint16_t  temps;          // temps du mvt en servomoteur / de rotation en moteur
  uint16_t  posMax;         // valeur extrème de la position que peut prendre ce moteur / ASC = 200
  uint16_t  posMin;         // valeur minimale de la position que peut prendre ce moteur
  byte      nbPos;          // nombre de positions prévues pour ce servo / pap
  uint32_t  hBouge;         // heure à laquelle on met en mouvement le servomoteur
};
#define nbActeur      10    // nombre maxi d'actionneurs dans l'ensemble du robot
Acteur Acteurs[nbActeur];   // tableau de toutes les données de tous les servomoteurs
Acteur acteur;              // élément en cours de manipulation
byte   actionneur;          // code de l'actionneur en cours d'emploi 
byte   grappe;              // son numéro de grappe
byte   pos;                 // et son code position courant ou visé
#define nbPositions   5     // nombre de positions maximales pour un actionneur
uint16_t Positions[nbActeur][nbPositions];  // positions angulaires pour servo et mm pour l'ascenseur

// dictionnaire des actionneurs
#define POMPE         0
#define PINCED        1
#define TOURNECOLG    2
#define TOURNECOLD    3
#define PINCEG        4
#define BANDEROLE     5
#define GLISSAIMANT   6
#define BASCPOMPE     7
#define TILTCAM       8
#define ASCENSEUR     9
#define EMBASE        12    // cas particulier car grappe et analogiques 

// différents type d'actionneurs  
#define SMLX16    0         // 8 servo numériques employés
#define MOTDC     1         // pompe 12v en mode marche/Arret via PWM d'où ce type
#define Pap42D    2         // utilisé pour l'ascenseur et l'embase
#define Pap42C    3         // non utilisé ici
#define Grappe    4         // les pinces et les tourneurs de pinces

//*** ASPECT GESTES
// On décrit ici une posture d'un actionneur. C'est à dire un couple actionneur/position
// Ce sera la base des séquences qui egreneront les postures les unes après les autres
#define   nbGestes    40
struct Geste
{
  byte     code;        // numéro de code de ce geste (pour l'instant numéro d'ordre)
  byte     actionneur;  // code de l'actionneur concerné (rang dans le tableau Acteurs)
  byte     position;    // code position de l'actionneur (du tableau Acteurs)
  String   libelle;     // texte de description
  bool     actif;       // indique que l'on est dans cette position dans la séquence
  uint16_t delai;       // nombre de ms nécessaire pour réaliser ce geste
};
Geste Gestes[nbGestes]; // stock tous des gestes possible
Geste gesteEnCours;     // désigne les données du geste en cours
byte  numGeste;         // numéro du geste en cours dans la séquence

//*** ASPECT SEQUENCES
#define PrendTribune  0
#define MonteTribune  1
#define nbEtapes      20
struct Sequence
{                     
  byte      nbGeste;      // nombre de gestes actifs (y compris grappes) dans cette séquence
  String    libelle;      // explication de cette séquence
  byte      etapes[nbEtapes];   // tableau des gestes successifs (pas d'étapes en parallèle : Grappes)
  byte      etapeEnCours; // numéro du geste en cours de réalisation
};
#define nbSequence 2        // pour l'instant, deux séquences identifiées comme à faire
Sequence Seq[nbSequence];   // le tableau des séquences construites
Sequence seqEnCours;        // la séquence en cours
byte sequence;              // numéro de la séquence en cours
bool seqPasFinie;           // vrai tant qu'une séquence est demandée et pas finie ou abandonnée
bool seqOk;                 // indique qu'une séquence évolue normalement, faux si suspension
byte numEtape;              // numéro de l'étape en cours dans la séquence
unsigned long hDebut;       // heure en ms de début du geste (sert à lancer l'étape suivante)     

/***********    FONCTIONS DES SERVOS - GESTES  - SEQUENCES    *********************/

void creeActeurs()
{     // cette fonction définie toutes les propriétés des actionneurs employés
  Acteurs[POMPE].adr     = 0;
  Acteurs[POMPE].libelle = "Pompe Planche";
  Acteurs[POMPE].type    = MOTDC;
  Acteurs[POMPE].pos     = 0;
  Acteurs[POMPE].temps   = 128;
  Acteurs[POMPE].posMax  = 255;   // plage de la PWM ou PWM de la tension max
  Acteurs[POMPE].nbPos   = 2;
  Positions[POMPE][0] = 0;
  if (BATTERIE == 20) 
    Positions[POMPE][1] = 160;   // pwm sur 255 pour avoir 12V à partir de 20
  if (BATTERIE == 15)
    Positions[POMPE][1] = 205;   // pwm sur 255 pour avoir 12V à partir de 15
  // second actionneur
  Acteurs[PINCED].adr     = 1;
  Acteurs[PINCED].libelle = "Pince Droite";
  Acteurs[PINCED].type    = SMLX16;
  Acteurs[PINCED].grappe  = 1;
  Acteurs[PINCED].posGrappe = 0;
  Acteurs[PINCED].mode    = modeServo;
  Acteurs[PINCED].pos     = 123;
  Acteurs[PINCED].temps   = 128;
  Acteurs[PINCED].posMax  = 1000;
  Acteurs[PINCED].nbPos   = 2;
  Positions[PINCED][0]    = 615;   // fermée
  Positions[PINCED][1]    = 852;   // ouverte
  // troisième actionneur
  Acteurs[TOURNECOLG].adr     = 2;
  Acteurs[TOURNECOLG].libelle = "Tourne Colonne Gauche";
  Acteurs[TOURNECOLG].type    = SMLX16;
  Acteurs[TOURNECOLG].grappe  = 2;
  Acteurs[TOURNECOLG].posGrappe = 1;
  Acteurs[TOURNECOLG].mode    = modeServo;
  Acteurs[TOURNECOLG].pos     = 123;
  Acteurs[TOURNECOLG].temps   = 128;
  Acteurs[TOURNECOLG].posMax  = 1000;
  Acteurs[TOURNECOLG].nbPos   = 2;
  Positions[TOURNECOLG][0]    = 231;   // face
  Positions[TOURNECOLG][1]    = 520;   // ecarte
  // quatrieme actionneur 
  Acteurs[TOURNECOLD].adr     = 3;
  Acteurs[TOURNECOLD].libelle = "Tourne Colonne Droite";
  Acteurs[TOURNECOLD].type    = SMLX16;
  Acteurs[TOURNECOLD].grappe  = 2;
  Acteurs[TOURNECOLD].posGrappe = 0;
  Acteurs[TOURNECOLD].mode    = modeServo;
  Acteurs[TOURNECOLD].pos     = 123;
  Acteurs[TOURNECOLD].temps   = 128;
  Acteurs[TOURNECOLD].posMax  = 1000;
  Acteurs[TOURNECOLD].nbPos   = 2;
  Positions[TOURNECOLD][0]    = 800;   // face
  Positions[TOURNECOLD][1]    = 300;   // ecarte
  // cinquieme actionneur 
  Acteurs[PINCEG].adr     = 4;
  Acteurs[PINCEG].libelle = "Pince Gauche";
  Acteurs[PINCEG].type    = SMLX16;
  Acteurs[PINCEG].grappe  = 1;
  Acteurs[PINCEG].posGrappe = 1;
  Acteurs[PINCEG].mode    = modeServo;
  Acteurs[PINCEG].pos     = 123;
  Acteurs[PINCEG].temps   = 128;
  Acteurs[PINCEG].posMax  = 1000;
  Acteurs[PINCEG].nbPos   = 2;
  Positions[PINCEG][0]    = 525;   // fermée
  Positions[PINCEG][1]    = 350;   // ouverte
  // sixième actionneur 
  Acteurs[BANDEROLE].adr     = 5;
  Acteurs[BANDEROLE].libelle = "Depose Banderole";
  Acteurs[BANDEROLE].type    = SMLX16;
  Acteurs[BANDEROLE].grappe  = 1;
  Acteurs[BANDEROLE].posGrappe = 0;
  Acteurs[BANDEROLE].mode    = modeServo;
  Acteurs[BANDEROLE].pos     = 123;
  Acteurs[BANDEROLE].temps   = 128;
  Acteurs[BANDEROLE].posMax  = 1000;
  Acteurs[BANDEROLE].nbPos   = 3;
  Positions[BANDEROLE][0]    = 800;   // range
  Positions[BANDEROLE][1]    = 300;   // pose
  Positions[BANDEROLE][2]    = 300;   // tire
  // septième actionneur 
  Acteurs[GLISSAIMANT].adr     = 6;
  Acteurs[GLISSAIMANT].libelle = "Glisseur Aimants";
  Acteurs[GLISSAIMANT].type    = SMLX16;
  Acteurs[GLISSAIMANT].grappe  = 0;
  Acteurs[GLISSAIMANT].posGrappe = 1;
  Acteurs[GLISSAIMANT].mode    = modeServo;
  Acteurs[GLISSAIMANT].pos     = 123;
  Acteurs[GLISSAIMANT].temps   = 128;
  Acteurs[GLISSAIMANT].posMax  = 1000;
  Acteurs[GLISSAIMANT].nbPos   = 2;
  Positions[GLISSAIMANT][0]    = 0;     // pique
  Positions[GLISSAIMANT][1]    = 500;   // largue
  // huitième actionneur 
  Acteurs[BASCPOMPE].adr     = 7;
  Acteurs[BASCPOMPE].libelle = "Bascule Pompe";
  Acteurs[BASCPOMPE].type    = SMLX16;
  Acteurs[BASCPOMPE].grappe  = 0;
  Acteurs[BASCPOMPE].posGrappe = 1;
  Acteurs[BASCPOMPE].mode    = modeServo;
  Acteurs[BASCPOMPE].pos     = 123;
  Acteurs[BASCPOMPE].temps   = 128;
  Acteurs[BASCPOMPE].posMax  = 1000;
  Acteurs[BASCPOMPE].nbPos   = 3;
  Positions[BASCPOMPE][0]    = 370;   // debout
  Positions[BASCPOMPE][1]    = 825;   // pique
  Positions[BASCPOMPE][2]    = 575;   // souleve
  // neuvième actionneur 
  Acteurs[TILTCAM].adr     = 8;
  Acteurs[TILTCAM].libelle = "Tilt Caméra";
  Acteurs[TILTCAM].type    = SMLX16;
  Acteurs[TILTCAM].grappe  = 0;
  Acteurs[TILTCAM].posGrappe = 1;
  Acteurs[TILTCAM].mode    = modeServo;
  Acteurs[TILTCAM].pos     = 123;
  Acteurs[TILTCAM].temps   = 128;
  Acteurs[TILTCAM].posMax  = 1000;
  Acteurs[TILTCAM].nbPos   = 3;
  Positions[TILTCAM][0]    = 370;   // balises
  Positions[TILTCAM][1]    = 825;   // pique
  Positions[TILTCAM][2]    = 575;   // souleve
  // dixième actionneur 
  Acteurs[ASCENSEUR].adr     = 4;     // à vérifier l'étiquette
  Acteurs[ASCENSEUR].libelle = "Ascenseur";
  Acteurs[ASCENSEUR].type    = Pap42D;
  Acteurs[ASCENSEUR].grappe  = 0;
  Acteurs[ASCENSEUR].temps   = 128;
  Acteurs[ASCENSEUR].posMax  = 200;
  Acteurs[ASCENSEUR].nbPos   = 5;
  Positions[ASCENSEUR][0]    = 0;     // Home
  Positions[ASCENSEUR][1]    = 2;     // +2mm
  Positions[ASCENSEUR][2]    = 10;    // +1cm
  Positions[ASCENSEUR][3]    = 125;   // +12.5mm
  Positions[ASCENSEUR][4]    = 140;   // +14cm
  // pour tous les servos créés, appeler la fonction LobotSerialServoLoadid(id);
}

void creeGestes()
{     // définition des tous les gestes que peuvent faire les actionneurs (29 !)
  for (byte g=0 ; g < nbGestes ; g++)
    Gestes[g].actif = false;    // aucun actif pour l'instant
  // pompe les 5 bits de poids fort = 0
  Gestes[0].code = 0;
  Gestes[0].actionneur = 0;
  Gestes[0].position = 0;
  Gestes[0].libelle = "Pompe éteinte";
  Gestes[1].code = 1;
  Gestes[1].actionneur = 0;
  Gestes[1].position = 1;
  Gestes[1].libelle = "Pompe Aspiration";
  // Pince droite : les 5 bits de poids fort = 1 = 8
  Gestes[2].code = 8;
  Gestes[2].actionneur = 1;
  Gestes[2].position = 0;
  Gestes[2].libelle = "Pince Droite fermée";
  Gestes[3].code = 9;
  Gestes[3].actionneur = 1;
  Gestes[3].position = 1;
  Gestes[3].libelle = "Pince Droite ouverte";
  // Pince gauche : les 5 bits de poids fort = 4 = 32
  Gestes[4].code = 32;
  Gestes[4].actionneur = 4;
  Gestes[4].position = 0;
  Gestes[4].libelle = "Pince Gauche fermée";
  Gestes[5].code = 33;
  Gestes[5].actionneur = 4;
  Gestes[5].position = 1;
  Gestes[5].libelle = "Pince Gauche ouverte";
  // Tourne Pince droite : les 5 bits de poids fort = 3 = 24
  Gestes[6].code = 24;
  Gestes[6].actionneur = 3;
  Gestes[6].position = 0;
  Gestes[6].libelle = "Pince Droite coté face";
  Gestes[7].code = 25;
  Gestes[7].actionneur = 3;
  Gestes[7].position = 1;
  Gestes[7].libelle = "Pince Droite écartée";
  // Tourne Pince Gauche : les 5 bits de poids fort = 2 = 16
  Gestes[8].code = 16;
  Gestes[8].actionneur = 2;
  Gestes[8].position = 0;
  Gestes[8].libelle = "Pince Gauche coté face";
  Gestes[9].code = 17;
  Gestes[9].actionneur = 2;
  Gestes[9].position = 1;
  Gestes[9].libelle = "Pince Gauche écartée";
  // Glisse Aimants : les 5 bits de poids fort = 6 = 48
  Gestes[10].code = 48;
  Gestes[10].actionneur = 6;
  Gestes[10].position = 0;
  Gestes[10].libelle = "Aimants Face pour Prise";
  Gestes[11].code = 49;
  Gestes[11].actionneur = 6;
  Gestes[11].position = 1;
  Gestes[11].libelle = "Aimants Coté pour Larguer";
  // Banderole les 5 bits de poids forts = 5 : 40
  Gestes[12].code = 40;
  Gestes[12].actionneur = 5;
  Gestes[12].position = 0;
  Gestes[12].libelle = "Banderole Rangée";
  Gestes[13].code = 41;
  Gestes[13].actionneur = 5;
  Gestes[13].position = 1;
  Gestes[13].libelle = "PoseBanderole";
  Gestes[14].code = 42;
  Gestes[14].actionneur = 5;
  Gestes[14].position = 2;
  Gestes[14].libelle = "Lache Banderole";
  // Bascule Ventouses les 5 bits de poids forts = 7 : 56
  Gestes[15].code = 56;
  Gestes[15].actionneur = 7;
  Gestes[15].position = 0;
  Gestes[15].libelle = "Ventouses Levées";
  Gestes[16].code = 57;
  Gestes[16].actionneur = 7;
  Gestes[16].position = 1;
  Gestes[16].libelle = "Ventouses en Prise";
  Gestes[17].code = 58;
  Gestes[17].actionneur = 7;
  Gestes[17].position = 2;
  Gestes[17].libelle = "Ventouses Souleve";
  // Tilt Camera les 5 bits de poids forts = 8 : 64
  Gestes[18].code = 64;
  Gestes[18].actionneur = 8;
  Gestes[18].position = 0;
  Gestes[18].libelle = "Camera sur Balises";
  Gestes[19].code = 65;
  Gestes[19].actionneur = 8;
  Gestes[19].position = 1;
  Gestes[19].libelle = "Caméra à -20";
  Gestes[20].code = 66;
  Gestes[20].actionneur = 8;
  Gestes[20].position = 2;
  Gestes[20].libelle = "Caméra à -40";
  // Ascenseur les 5 bits de poids forts = 9 : 72
  Gestes[21].code = 72;
  Gestes[21].actionneur = 9;
  Gestes[21].position = 0;
  Gestes[21].libelle = "Ascenseur en 'Home'";
  Gestes[22].code = 73;
  Gestes[22].actionneur = 9;
  Gestes[22].position = 1;
  Gestes[22].libelle = "Ascenseur à 2mm";
  Gestes[23].code = 74;
  Gestes[23].actionneur = 9;
  Gestes[23].position = 2;
  Gestes[23].libelle = "Ascenseur à 1cm";
  Gestes[24].code = 75;
  Gestes[24].actionneur = 9;
  Gestes[24].position = 3;
  Gestes[24].libelle = "Ascenseur à 12.5cm";
  Gestes[25].code = 75;
  Gestes[25].actionneur = 9;
  Gestes[25].position = 4;
  Gestes[25].libelle = "Ascenseur à 14cm";
  // on s'occupe maintenant des grappes
  // PINCES : les 5 bits de poids fort = 10 = 80
  Gestes[26].code = 80;
  Gestes[26].actionneur = 10;
  Gestes[26].position = 0;
  Gestes[26].libelle = "Pinces fermées";
  Gestes[27].code = 81;
  Gestes[27].actionneur = 10;
  Gestes[27].position = 1;
  Gestes[27].libelle = "Pinces ouvertes";
  // TOURNES PINCES : les 5 bits de poids fort = 11 = 88
  Gestes[28].code = 88;
  Gestes[28].actionneur = 11;
  Gestes[28].position = 0;
  Gestes[28].libelle = "Pinces en face";
  Gestes[29].code = 89;
  Gestes[29].actionneur = 11;
  Gestes[29].position = 1;
  Gestes[29].libelle = "Pinces écartées";
}

void creeSequences()
{     // création de la liste des séquences 
  Seq[PrendTribune].nbGeste = 8;
  Seq[PrendTribune].libelle = "Prend une tribune";
  Seq[PrendTribune].etapeEnCours = 255;     // indique qu'aucune étape est en cours
  Seq[PrendTribune].etapes[0] = 21;
  Seq[PrendTribune].etapes[1] = 10;
  Seq[PrendTribune].etapes[2] = 28;
  Seq[PrendTribune].etapes[3] = 27;
  Seq[PrendTribune].etapes[4] = 15;
  Seq[PrendTribune].etapes[5] = 32;  // ou 33 selon les mesures (à adapter dans le lancement) 
                // à répéter éventuellement jusqu'à 2 échos voisins
                // avant cette étape 6, il faut mesurer la distance et paramétrer l'avance
  Seq[PrendTribune].etapes[6] = 26;
  Seq[PrendTribune].etapes[7] = 23;

  Seq[MonteTribune].nbGeste = 18;
  Seq[MonteTribune].libelle = "Assemble une tribune";
  Seq[PrendTribune].etapeEnCours = 255;     // indique qu'aucune étape est en cours
  Seq[MonteTribune].etapes[0] = 29;
  Seq[MonteTribune].etapes[1] = 22;
  Seq[MonteTribune].etapes[2] = 11;
  Seq[MonteTribune].etapes[3] = 1;
  Seq[MonteTribune].etapes[4] = 16;
  Seq[MonteTribune].etapes[5] = 17;
  Seq[MonteTribune].etapes[6] = 21;
  Seq[MonteTribune].etapes[7] = 30;
  Seq[MonteTribune].etapes[8] = 24;
  Seq[MonteTribune].etapes[9] = 31;  
  Seq[MonteTribune].etapes[10] = 28;
  Seq[MonteTribune].etapes[11] = 16;
  Seq[MonteTribune].etapes[12] = 0;
  Seq[MonteTribune].etapes[13] = 27;
  Seq[MonteTribune].etapes[14] = 15;
  Seq[MonteTribune].etapes[15] = 30;
}

/******************     FONCTIONS  POSTURES  ET  MOUVEMENTS    *****************/

void activeLX16()
{     // rend opérationnels tous les servomoteurs de type LX16 du robot
  for(byte act = 0; act < nbActeur; act++)
  {
    if (Acteurs[act].type == SMLX16)
      LobotSerialServoLoad(Acteurs[act].adr);
  }
}

uint16_t litPos(byte adrServo)
{ // cette fonction retourne la position courante du servo dont on donne l'adresse
  uint16_t posLue;
  posLue = repeteLecture(adrServo, LOBOT_SERVO_POS_READ, 5);
  if (posLue > 0 )    return (posLue);
  else                return (65535);
}

void poseServo (byte code, byte pos)
{   // cette fonction met le servomoteur 'code' dans la position associée au code 'pos'
  // maintenant qu'on connaît la valeur angulaire, on pilote vraiment ce servo
  LobotSerialServoMove(Acteurs[code].adr, Positions[code][pos], Acteurs[code].temps);
  Acteurs[code].pos = Positions[code][pos];
  Acteurs[code].codePos = pos;
  Acteurs[code].hBouge = millis();
}

/***************    FONCTIONS DE PILOTAGE PAR LA CONSOLE INUTILES ICI   *************/
/*
uint16_t limiteChangement(uint8_t adrServo, int16_t posDemandee)
{   // cette fonction recoit une position demandée à un servo et retroune la position limitée
  bool trouve = false;
  for (byte servo = 0; servo < nbSMMax; servo++)  // on explore les servos de la servos
  {
    if (servos[servo].adr == adrServo)     // si on a trouvé le servo à changer
    {
      trouve = true;
      if (posDemandee < servos[servo].posMin) posDemandee = servos[servo].posMin;
      if (posDemandee > servos[servo].posMax) posDemandee = servos[servo].posMax;
    }
  }
  if (!trouve) posDemandee = 65535;     // pour indiquer résultat inexploitable car servo non trouvé
  return posDemandee;
}

void changePos(byte adrServo, int8_t change)
{   // cette fonction déplace le servo sélectionné de l'amplitude codée 
  int16_t bouge;
  byte code = codeAdresse(adrServo); // on cherche le servo qui est concerné dans le tableau
  if (code != 255)
  {
    bouge = limiteChangement(adrServo, servos[code].pos + change);
    LobotSerialServoMove(adrServo, bouge, 100);   // on met la position demandée ou limite
  }
}

byte litCommande()
{   // cette fonction lit une commande sur une lettre au clavier console et l'exécute
    // elle retourne l'adresse du servomoteur choisi
  static byte adrServoActionne;
  static byte grappeChoisie;
  char c = Serial.read();
  switch (c)
  {
    case '1' : adrServoActionne = 1; 
              Serial.println("Maintenant on agit sur SERVO 1"); break;
    case '2' : adrServoActionne = 2; 
              Serial.println("Maintenant on agit sur SERVO 2"); break; 
    case '3' : adrServoActionne = 3; 
              Serial.println("Maintenant on agit sur SERVO 3"); break;
    case '4' : adrServoActionne = 4; 
              Serial.println("Maintenant on agit sur SERVO 4"); break; 
    case '5' : adrServoActionne = 5; 
              Serial.println("Maintenant on agit sur SERVO 5"); break; 
    case 'a' : grappeChoisie = 0; 
              Serial.println("Maintenant on agit sur LA GRAPPE 0"); 
              afficheGrappe(0); break;
    case 'b' : grappeChoisie = 1; 
              Serial.println("Maintenant on agit sur LA GRAPPE 1"); 
              afficheGrappe(1); break;
    case '+' : changePos(adrServoActionne, 5); // incrément de 5 de la position du servo actionné
              Serial.printf ("On bouge le servo %d de +5  \n", adrServoActionne); break;
    case '-' : changePos(adrServoActionne, -5);  // diminution de 5 de la position du servo actionné
              Serial.printf ("On bouge le servo %d de -5  \n", adrServoActionne); break;
    case '*' : changePos(adrServoActionne, 30);  // incrément de 30 de la position du servo actionné
              Serial.printf ("On bouge le servo %d de +30  \n", adrServoActionne); break;
    case '/' : changePos(adrServoActionne, -30);  // diminution de 30 de la position du servo actionné
              Serial.printf ("On bouge le servo %d de -30  \n", adrServoActionne); break;
  }
  return adrServoActionne;
}
*/
/***************    FONCTIONS DE PILOTAGE DE LA POMPE  *****************/

void Pompe(bool cde)
{
  if (cde)                                       // si on la fait tourner
  {
    analogWrite (CDEPOMPE, Positions[POMPE][1]);    // on met la PWM retenué sur le signal
    Acteurs[POMPE].pos = Positions[POMPE][1];           // on enregistre l'état demandé
    Acteurs[POMPE].codePos = 1;                     // on enregistre l'état demandé
  }
  else                                          // sinon, on l'arrête
  {
    analogWrite (CDEPOMPE, Positions[POMPE][0]);    // on met la PWM retenué sur le signal
    Acteurs[POMPE].pos = Positions[POMPE][0];           // on enregistre l'état demandé
    Acteurs[POMPE].codePos = 0;                     // on enregistre l'état demandé
  }
  Acteurs[POMPE].hBouge = millis();             // on enregistre l'heure de cette action
}

byte EtatPompe ()
{
  return (Acteurs[POMPE].pos);      // on retourne l'élément le plus précis de son état
}

/*****************    FONCTIONS DE  PILOTAGE  DES  STEPPERS   ***********/

void initAscenseur()
{   // cette fonction initialise le fonctionnement de l'accès au 'HOME' de l'ascenseur
  // il faut tester les valeurs de DESCEND et de TRIG_HOME
  // on utilise : set_home_parameters(int8_t moteur_index, uint8_t trig, uint8_t dir, uint16_t speed)
  set_home_parameters(ascenseur, TRIG_HOME, DESCEND, 50);
  if (DEBUGASC)
    Serial.println ("Initialisation de l'ascenseur en sens et polarité du 'Home'");
}

void arreteStepper(byte adr)
{  // on va envoyer sur le bus RS 485 la commande pour le Servo42D correpondante

}

uint32_t questionneAscenseur()
{   // cette fonction demande le code de la position actuelle de l'ascenseur. 
    // on utilise lit_reponse_variable(uint8_t code_attendu, std::vector<uint8_t>& data_out, uint8_t taille_donnee_attendue)
  std::vector<uint8_t> posit;             // valeur de retour renseignée
  uint32_t positionActuelle = 0;          // résultat qui est élaboré
  bool ok = lire_reponse_variable(CODE_TRAME[static_cast<uint8_t>(Commande::DemanderPosition)], posit, 4, ascenseur);
  if (ok)
  {
    positionActuelle = posit[3]<<24 + posit[2]<<16 + posit[1]<<8 + posit[0];    // on remet dans l'ordre les octets de la réponse
    Acteurs[ASCENSEUR].papPos = positionActuelle;                               // on enregistre cette positions dans le tableau des actionneurs
    if (DEBUGASC)
      Serial.printf ("Position actuelle de l'asscenseur %d \n", positionActuelle);  // on affiche la position
  }
  else positionActuelle = 0xFFFFFFF0;                                             // valeur débile de retour impossible
  return (positionActuelle);
}

void deplaceAscenseur(byte nouvellePosition)
{   // cette fonction positionne l'ascenceur à la hauteur en mm dont le code est le paramètre
  // on utilise : MouvRelPulse(uint8_t adr_moteur, uint8_t direction, uint16_t vitesse, uint8_t acceleration, uint32_t impulsions);
  bool sens;
  uint32_t posActuelle, posAttendue;
  int32_t delta;
  posActuelle = questionneAscenseur();                 // on lit la position actuelle en impulsions
  posAttendue = Positions[ASCENSEUR][nouvellePosition] * pasParMm;  // numéro de pas à atteindre
  if (DEBUGASC)
    Serial.printf ("On va à la position %d soit le numéro de pas : %d\n", nouvellePosition, posAttendue);
  if (posActuelle != 0xFFFFFFF0)                      // si la position est connue  
  {
    delta = Positions[ASCENSEUR][nouvellePosition] - posActuelle; // on calcule le nb de pas à faire
    sens = (delta < 0);                               // on détermine le sens de rotation
    if (DEBUGASC)
      Serial.printf ("Cela représente: %d pas\n", nouvellePosition, posAttendue);
    MouvRelPulse (ascenseur, sens, 300, 0, abs(delta));    // et on commande le déplacement
    Acteurs[ASCENSEUR].hBouge = millis();             // on enregistre l'heure de cette action
  }
  else 
    if (DEBUGASC)   
      Serial.println (" Mesure de position impossible -> déplacement impossible");
}

void deplaceRobot (uint16_t dx, uint16_t dy, uint16_t dc)
{
  uint8_t vit_nominale = 20;
  trimoteur_dep (dx, dy, dc, vit_nominale);    // pas d'accélération
}

/******************    FONCTIONS  RELATIVES AUX SEQUENCES   *********************/

void etapeAction()
{       // cette fonction, appellée à chaque changement d'étape recherche l'action à mener et l'exécute
  numGeste = seqEnCours.etapes[numEtape];
  gesteEnCours = Gestes[numGeste];
  acteur = Acteurs[gesteEnCours.actionneur];
  if (DEBUGSEQ)
    Serial.printf ("Nouvelle action : %s \n", gesteEnCours.libelle.c_str());
  // affichage de type et de libellé
  hDebut = millis();                      // on enregistre l'heure du début du geste
  switch (acteur.type)                    // selon l'actionneur, on fait appel à des fonctions différentes
  {
    case SMLX16 :       // si c'est un servomoteur isolé
                          // on lit le code position à atteindre
                          // petit message pour expliquer ce qu'on fait
                          // on en déduit la position angulaire à atteindre
                          // on envoie la commande au servomoteur
      break;
    case Grappe :       // si c'est une grappe de servomoteurs
                          // on lit le code position à atteindre
                          // peit message pour expliquer ce qu'on fait
                          // on explore le tableau des acteurs
                            // si on trouve un acteur de la grappe 
                            // on en déduit la position angulaire à placer
                            // on envoie la commande à chaque servo de la grappe 
      break;
    case MOTDC :        // si c'est la pompe
                          // on lit le code position à atteindre
                          // petit message pour expliquer ce qu'on fait
                          // on en déduit la position angulaire à atteindre
                          // on envoie la commande au servomoteur
      break;
    case Pap42D :       // si c'est l'ascenseur ou l'embase
                          // on lit le code position à atteindre
                          // petit message pour expliquer ce qu'on fait
                          // on en déduit la position angulaire à atteindre
                          // on envoie la commande au servomoteur
      break;
  }
}

void lanceSequence(byte natureSeq)
{       // cette fonction initialise les données de la séquence demandée
  if (natureSeq == 0xFF)            // si c'est un ordre d'arrêt ou de suspension
    if (numEtape > nbEtapes)   seqPasFinie = false;    // cela veut dire Annulation !!
  else
  {
    if (natureSeq == 'P')       // si on est en séquence "Prise du matériel Tribune"
      seqEnCours = Seq[PrendTribune];
    if ((natureSeq == 'C') && seqPasFinie)  // si on construit une tribune
      seqEnCours = Seq[MonteTribune];           
    seqPasFinie = true;                     // maintenant la séquence a commencé
    numEtape = 0;                           // on est à la première étape
    gesteEnCours =  Gestes[seqEnCours.etapes[numEtape]];   // on a trouvé le geste à faire
    gesteEnCours.actif = true;              // on rend ce geste actif
    if (gesteEnCours.code > nbGestes)       // si le code est aberrant
      Serial.print ("ALERTE CODE GESTE IMPOSSIBLE");
    else
      etapeAction();                          // va exécuter la première étape
  }
}

void derouleSequence (bool ok)
{       // cette fonction suspend le déroulement temporel de la séquence ou la reprend selon 'ok'
  if ((millis() > (hDebut + gesteEnCours.delai)) && ok)          // s'il est l'heure de faire le geste suivant
  {
    // rendre inactif le geste et 
    numEtape++;                         // on passe à l'étape suivante
    if (numEtape > seqEnCours.nbGeste)  // si on a dépassé le nombre de gestes à faire
      seqPasFinie = false;              // c'est fini
    else
    {
      gesteEnCours =  Gestes[seqEnCours.etapes[numEtape]];   // geste en cours est la nouvelle étape de la séquence
      gesteEnCours.actif = true;        // rendre actif le geste
      etapeAction();                    // que l'on exécute
    }
  }
}

/****************   FONCTIONS DE TEST  ***********************/

void testAscenceur()
{         // on va aller du home à 1cm puis à 12.5cm et retour à home
  go_home(ascenseur);                   // on va en home
  questionneHome(ascenseur, 5000, 200); // on vérifie qu'il est arrivé (en 5s)
  delay (3000);
  deplaceAscenseur(2);                  // on va à la position codée par 2 : 1cm
  delay(1000);
  deplaceAscenseur(3);                  // on va à la psoition codée par 3 : 14cm
  delay (3000);
}

#endif