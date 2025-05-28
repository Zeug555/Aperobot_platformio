#ifndef USB_TRAMES_H
#define USB_TRAMES_H

/**************************************************************************
* Cette petite bibliothèque définit les structures, variables et fonctions
* qui permettent à une Nucléo L476RG d'employer son port USB en relation 
* avec un système informatique de pilotage de robot qui aiguille vers deux 
* ports série UART les commandes et les réponses d'un bus RS485 ( pour la
* liaison aux moteurs pas à pas) et un bus 1 fil (pour des servomoteurs de 
* type LX16A).
****************************************************************************
* Trames de commandes Actionneurs : [Stx] [Cde] [Act] [Pos] [CRC] [Etx]
* Commandes : 'P' = positionne / 'E' = demande d'état
* Réponses  si cde = P : Ack ou Nak  / si cde = E : [Stx] 'E' [Pos] crc [Etx] 
* [Act] = code de l'actionneur 0..9 ou grappe A..B ou séquence P..C (dans ce
* dernier cas, la position est vide ' ')
****************************************************************************
* Trames de commande Moteurs PAP de l'embase : 
* [Stx] 'M' [M1_v] [M1_d] [M2_v] [M2_d] [M3_v] [M3_d] [CRC] [Etx]
* Réponses Ack ou Nak
****************************************************************************
* Trame de questionnement des capteurs logiques ET de distance : Stx 'F' Etx
* Réponse par Stx 'F' [ggg] [ddd] [b] [GG] [CG] [CD] [DD] crc Etx
* Trame de questionnement du capteur OTOS : [Stx] ['S'] [nnn] [crc] [Etx]
* Réponse : [Stx] 'S' [x] [y] [h] [vx] [vy] [vh] [ax] [ay] [ah] [CRC] [Etx] 
****************************************************************************
* Les valeurs CRC = somme des octets précédents (sans STX) et écrété à 255.
****************************************************************************/

#include <Arduino.h>
#include <stdint.h>
#include <servo42d.h>
#include <global.h>

//*** ASPECT COMMUNIICATION voie USB vers la RPI4

//*** CONSTANTES ASCII des trames
/*
#define ACK         6
#define NAK         21      // 0x15
#define STX         2
#define ETX         3
*/
// pour la partie debug des commandes via le moniteur arduino
#define ACK         '!'
#define NAK         '?'      
#define STX         '/'
#define ETX         '\\'
bool ordreRecu;             // indique qu'une trame est recue, la traiter
uint8_t numChar;            // nombre de caractères déja reçus par l'USB
char tamponRecep[30];       // tampon de réception depuis la RPI4 (la + longue = Steppers : 16)
char tamponEmis[42];        // tampon de la chaine émise vers la RPI4 (la + longue = OTOS : 40)

//*** TYPES DE TRAMES
#define Pilotage    0     // pilotages d'actionneurs (hors steppers)  'P'
#define DemandeEtat 1     // demande d'état des actionneurs           'E'
#define Deplace     2     // pilotage des moteurs pap de l'embase     'D'
#define Capteurs    3     // demande d'état des capteurs (hors OTOS)  'C'
#define OTOS        4     // demande d'état du capteur de position, de mouvements 'S'
#define Sequence    5     // lancement ou arret d'une séquence 'M'
byte typeOrdre;

/****************   FONCTIONS   UTILITAIRES    *****************/

bool valN(byte place, byte taille)
{   // cette fonction retourne vrai si les 'taille' caractères placés en 'place' dans le tampon sont des chiffres décimaux
  bool ok = true;
  for (byte pos = 0; pos < taille; pos++)
    if ((tamponRecep[place + pos] < '0') || (tamponRecep[place + pos] > '9'))
      ok = false;
  return (ok);
}

uint16_t enVal (byte place, byte taille)
{       // cette fonction traduit les 'taille' caractères placés en 'place' dans la chaine en un 0 < nombre < 65535
  uint16_t val = 0;
  for (byte pos = 0; pos < taille; pos++)
  {
    val = val + (tamponRecep[place + pos] - '0');
    if (pos < taille - 1)                           // pour ne pas multiplier par 10 les unités
      val *= 10;
  }
  return (val);
}

/****************   FONCTIONS   DE   DIALOGUE   USB    *****************/

void initSerials()
{     // initialise les trois ports UART employés à 115200 bauds 8N1
  Serial.begin (115200);
  BusLX16.begin (115200, SERIAL_8N1);
  if (PAP)
    BusPAP.begin (38400, SERIAL_8N1);
  delay(200);           // laisse le temps à la voie USB d'être ok
  if (DEBUG)
    Serial.println ("Ports série configurés");
}

void afficheTrameUtile ()
{     // affiche les caractères reçus s'ils sont imprimables
  byte posCar= 0;
  while (tamponRecep[posCar] != ETX)      // on explore la chaine recue
  {
    if ((tamponRecep[posCar] >= 0x20) && (tamponRecep[posCar] <= 0x7F)) // si c'est un code ASCII
      Serial.write (tamponRecep[posCar]);  // on l'affiche sur la console
    posCar++;
  }
  Serial.write (ETX);
}

void lit_USB()
{       // principale fonction : Lecture du port ; détection des délimiteurs ; indique la fin de réception
  if (Serial.available())
  {
    while (Serial.available())
    {
      char c = Serial.read();
      switch (c)
      {
        case STX :
          numChar = 0;
          tamponRecep[numChar] = c;
          break;
        case ETX :
          ordreRecu = true;       // une trame finie de recevoir à traiter
          grappe = 0;             // 'initialise' l'actionneur en cours de pilotage
          actionneur = 255;
          pos = 0;
          tamponRecep[++numChar] = c;
          break;
        default :
          tamponRecep[++numChar] = c;   // pas un délimiteur : on accumule
          break;
      }
    }
  }
}

/**************     FONCTIONS   DE   REPONSES  ET  DE   TRAITEMENTS   DES  ORDRES  **************/

void chercheOrganeDeGrappe(byte gr)
{     // positionne un actionneur appartenant à cette grappe
  for (byte act = 0; act < nbActeur; act++)
  {
    if (Acteurs[act].grappe = gr)
      actionneur = act;
  }
}

void pilote ()
{     // positionne l'actionneur demandé dans l'état codé reçu
  if (grappe == 0)                    // positionné dans testOrgane()
  {
    switch (Acteurs[actionneur].type) // selon le type d'actionneur concerné
    {
      case SMLX16 :     // si c'est un servomoteur isolé
        poseServo(actionneur, pos);   // on le met à la position
        break;
      case MOTDC :      // si c'est la pompe (seul moteur DC dans Aperobot)
        Pompe (pos);    // on va la mettre dans l'état demandé
        break;
      case Pap42D :     // si c'est l'ascenseur
        deplaceAscenseur(pos);
        /*
        // on calcule tous les paramètres de la commande de ce Servo 42D
        uint32_t nbImp = (Positions[9][pos] * pasParMm) - Acteurs[9].pos;
        bool dir = true;
        if (nbImp < 0)
        {
          nbImp = - nbImp;
          dir = ! dir;
        }
        // appel de la fonction de mise en position à vitesse donnée avec acceleration
        MouvRelPulse(Acteurs[9].adr, dir, vitNomAsc, 1, nbImp) ;
        */
        Acteurs[9].pos = Positions[9][pos];                     // on enregistre la nouvelle position
        Acteurs[9].codePos = pos;                               // et son code
        break;
    }
  }
  else    // si c'est une grappe
  {
    for (byte act = 0 ; act < nbActeur; act++)        // on explore le tableau des acteurs
    {
      if (Acteurs[act].grappe == grappe)      // si l'acteur appartient à cette grappe
        poseServo(Acteurs[act].adr, pos);     // on le commande
    }
  }
}

void ecrit(byte pos, byte nbcar, uint16_t val)
{       // écrit dans le tampon la valeur en ASCII à l'emplacement du 1er parametre
  char c;
  switch (nbcar)
  {
    case 4:
      c = (val / 1000) + '0';
      tamponEmis[pos++] = c;
    case 3 :
      c = ((val % 1000) / 100) +'0';
      tamponEmis[pos++] = c;
    case 2 :
      c = ((val % 100) / 10) +'0';
      tamponEmis[pos++] = c;
    case 1 : 
      c = (val % 10) + '0';
      tamponEmis[pos++] = c;
  }
}

void envoiTampon (byte taille)
{     // cette fonction calcule le CRC des 'taille' premiers caractères et finit la trame qu'elle émet
  uint16_t val = 0;
  for (byte c = 1; c < taille; c++)   // explore tout sauf STX
    val += tamponEmis[c];             // crc = somme des octets
  val &= 0xFF;                        // limité à 1 octet : 0xFF
  val |= 0x80;                        // pour rendre le CRC hors des caractères ASCII
  byte crc = val;                     // seul caractère non ascii de la trame
  tamponEmis[taille++] = crc;
  tamponEmis[taille] = ETX;
  for (byte c = 0; c < taille; c++ )
    Serial.write (tamponEmis[c]);
}

void interroge()
{     // cette fonction lit les capteurs sauf OTOS et construit la trame réponse
  // lecture des états des entrées logiques
  byte octet = construitCaptLogiques();
  char hexaine = (octet >> 4) + '0';   // traduction de l'hexa en ASCII
  char unite = (octet & 0xF) + '0';   // des deux caractères
  // Lecture des capteurs de fin de planche
  uint16_t ddd = litAnaCaptG();
  uint16_t ggg = litAnaCaptD();
  // lecture des 4 capteurs d'obstacle
  lit4CaptObstacle();
  // création et envoi de la trame de réponse
  tamponEmis[0] = STX;
  tamponEmis[1] = 'C';
  ecrit(2, 3, ddd);
  ecrit(5, 3, ggg);
  ecrit(8, 2, captAnas[captGG].lecture);
  ecrit(10, 2, captAnas[captGC].lecture);
  ecrit(12, 2, captAnas[captDC].lecture);
  ecrit(14, 2, captAnas[captDD].lecture);
  envoiTampon(16);
}

void otos()
{       // cette fonction programme le rythme des interruptions temporelles d'envoi des données otos
  if (valN(2, 3))
  {
    rythmeOtos = enVal(2, 3);    // il faut décoder les 3 chiffres qui suivent
    if (rythmeOtos == 0)    // si c'est '000' on désamorce le timer
      MyTimOtos->pause();
    else 
    {
      MyTimOtos->setOverflow(rythmeOtos*1000, MICROSEC_FORMAT);
      MyTimOtos->resume();
    }
  }
}

void envoiOtos()
{    // cette fonction lit le capteur, encode la trame et l'envoie à la RPI  (routine d'interruptions)
  Otos_read();      // lecture du capteur. sont positionnées les variables
  // on construit la trame de réponse OTOS 
  tamponEmis[0] = STX; tamponEmis[1] = 'S';
  // maintenant il faut décortiquer les valeurs de chaque groupe en chaines de caractere
  // position : pos.x ; pos.y ; pos.h
  ecrit(2, 4, int(posit.x * 1000));   // coordonnées du robot
  ecrit(6, 4, int(posit.y * 1000));
  ecrit(10, 4, int(posit.h * 1000));  // Cap
  // vitesse : vel.x ; vel.y ; vel.h 
  ecrit(14, 4, int(vel.x * 1000));
  ecrit(20, 4, int(vel.y * 1000));
  ecrit(24, 4, int(vel.h * 1000));  
  // acceleration : acc.x ; acc.y ; acc.h
  ecrit(28, 4, int(vel.x * 1000));
  ecrit(32, 4, int(vel.y * 1000));
  ecrit(36, 4, int(vel.h * 1000)); 
  envoiTampon (39);                 //puis l'envoyer
}

void envoiEtat(byte val)
{     // cette fonction envoie la trame vers RPI de la position d'un actionneur demandé
  tamponEmis[0] = STX;
  tamponEmis[1] = 'E';
  if (grappe == 0)    tamponEmis[2] = actionneur + '0';
  else                tamponEmis[2] = grappe - 1 + 'A';
  if (val == 'X')     tamponEmis[3] = val;
  else                tamponEmis[3] = val + '0';
  tamponEmis[4] = (tamponEmis[3]+tamponEmis[2]+tamponEmis[1]);
  tamponEmis[4] &= 0xFF;
  tamponEmis[4] |= 0x80;
  tamponEmis[5] = ETX;
  for (byte x = 0; x < 6; x++)
    Serial.write (tamponEmis[x]);
}

void etatSequence()
{       // cette fonction construit et envoie la trame d'état d'une séquence
  tamponEmis[0] = STX;
  switch (sequence)
  {
    case PrendTribune : 
      tamponEmis[1] = 'P';
      tamponEmis[2] = '1'; // si natureSeq = 'P', on envoie '1'
      break;
    case MonteTribune :
      tamponEmis[1] = 'C';
      if (numEtape <= 9)    tamponEmis[2] = '2';  // si numEtape <= 9 '2'
      else                  tamponEmis[2] = '3';  // sinon '3
      break;
  }
  tamponEmis[3] = (numEtape % 10) + '0';        // on met l'unité de numEtape
  tamponEmis[4] = (tamponEmis[1] + tamponEmis[2] + tamponEmis[3]);  // calcul du crc
  tamponEmis[4] &= 0xFF;
  tamponEmis[4] |= 0x80;
  tamponEmis[5] = ETX;                          // ajout de Etx
  for (byte x = 0; x < 6; x++)
    Serial.write (tamponEmis[x]);             // envoi de la trame de réponse
}

void questionne()
{     // cette fonction questionne les codes position des actionneurs et envoi la réponse
  uint16_t val = 0;                 // résultat à transmettre
  // il faut traiter à part les séquences car le format de la réponse est différent
  if ((tamponRecep[2] == 'P') || (tamponRecep[2] == 'C'))
    etatSequence();
  else
  {
    if (grappe == 0)
    {
      switch (Acteurs[actionneur].type) // selon le type d'actionneur concerné
      {
        case SMLX16 :     // si c'est un servomoteur isolé
          // on s'interroge s'il est en mouvement (délai action non écoulé)
          if (millis() < (Acteurs[actionneur].hBouge + Acteurs[actionneur].temps))
            val == 'X';   // dans ce cas, on renvoie 'X'
          else
            val = Acteurs[actionneur].codePos;                    // ou son code
          break;
        case MOTDC :      // si c'est la pompe
          val = EtatPompe();    // on va lire son état
          break;
        case Pap42D :     // si c'est l'ascenseur
          // on calcule tous les paramètres de la trame de questionnement de ce Servo 42D
          //val = Acteurs[actionneur].pos;                     // on transmet la position actuelle
          val = Acteurs[actionneur].codePos;                    // ou son code
          break;
      }
    }
    else    // si c'est une grappe
    {
    for (byte act = 0 ; act < nbActeur; act++)        // on explore le tableau des acteurs
      if (Acteurs[act].grappe == grappe)      // si l'acteur appartient à cette grappe
        val = Acteurs[act].codePos;     // on récupère son code de position
    }
    envoiEtat(val);
  }
}

void bougeEnVitesse()
{     // cette fonction doit traduire les ordres de vitesse et accel de la trame recue en ordres sur le bus RS485
  // rappel : La commande de chaque moteur se fait par envoyer_vitesse(mot[mot_droit], sens, vitesse, acc_globale);
  uint16_t vit1 = enVal(2,3);
  uint16_t vit2 = enVal(6,3);
  uint16_t vit3 = enVal(10,3);
  byte sens1 = tamponRecep[5] - '0';
  byte sens2 = tamponRecep[9] - '0';
  byte sens3=  tamponRecep[13] - '0';
  envoyer_vitesse(mot[mot_droit], sens3, vit3, acc_globale);
  envoyer_vitesse(mot[mot_gauche], sens2, vit2, acc_globale);
  envoyer_vitesse(mot[mot_arriere], sens1, vit1, acc_globale);
}

/************     FONCTIONS  DE  CONTROLE  ET  D'INTERPRETATION  DES  COMMANDES    **************/

bool testMouvement()
{     // cette fonction analyse la trame d'un ordre de mouvement en vitesse
  // rappel de la trame : [Stx] [Cde] [M1_v] [M1_d] [M2_v] [M2_d] [M3_v] [M3_d] [CRC] [Etx]
  bool ok = true; 
  if (valN(2,3) == false)                                     ok = false;
  if ((tamponRecep[5] != '0') && (tamponRecep[5] != '1'))     ok = false;
  if (valN(6,3) == false)                                     ok = false;
  if ((tamponRecep[9] != '0') && (tamponRecep[9] != '1'))     ok = false;
  if (valN(10,3) == false)                                    ok = false;
  if ((tamponRecep[13] != '0') && (tamponRecep[13] != '1'))   ok = false;
  return (ok);
}

bool testSequence()
{     // prépare une des séquences ou la suspend, la reprend, l'élimine
  bool ok = false;
  switch (tamponRecep[2])
  {     
    case 'P' :               // si c'est la séquence 1 : Prise de la Tribune
      sequence = PrendTribune;     
      seqOk = true;
      ok = true;
      break;
    case 'C' :              // si c'est la séquence 2 : Construit Tribune
      sequence = MonteTribune;
      seqOk = true;
      ok = true;
      break;
    case 'A' :              // si c'est l'arrêt, annulation de la séquence
      seqOk = false;
      sequence = 0xFF;
      numEtape = nbEtapes + 1;    // cela va arrêter la séquence
      ok = true;
      break; 
    case 'S':               // si c'est une suspension de la séquence
      seqOk = false;        // on ne va plus faire évoluer les gestes
      sequence = 0xFF;
      ok = true;
      break;
    case 'R':               // si c'est une reprise de la séquence suspendue
      seqOk = true,
      sequence = 0xFF;
      ok = true;
      break;
  }
  return (ok);
}

bool testOrgane()
{     // trouve l'organe ou la grappe qui doit être piloté ou questionné
  bool ok = false;
  byte organe = tamponRecep[2] - '0';
  // on teste si c'est un actionneur isolé
  if ((organe >= 0) && (organe < nbActeur))
  {
    ok = true;
    grappe = 0;
    actionneur = tamponRecep[2] - '0';
    if (DEBUGUSB)   // afficher son numéro et le libellé correspondant
      Serial.printf("organe %d actionneur %s ", actionneur, Acteurs[actionneur].libelle.c_str());
  }
  // il faut tester si c'est une grappe
  switch (tamponRecep[2])
  {
    case 'A' :           
      grappe = 1;
      ok = true;
      chercheOrganeDeGrappe(grappe);// dans ce cas, chercher dans Acteurs[] un organe associé
      if (DEBUGUSB)   // afficher son numéro et le libellé correspondant
        Serial.printf("grappe A tel que %d actionneur %s ", actionneur, Acteurs[actionneur].libelle.c_str());
      break;
    case 'B' :
      grappe = 2;
      ok = true;
      chercheOrganeDeGrappe(grappe);// dans ce cas, chercher dans Acteurs[] un organe associé
      if (DEBUGUSB)   // afficher son numéro et le libellé correspondant
        Serial.printf("grappe B tel que %d actionneur %s ", actionneur, Acteurs[actionneur].libelle.c_str());
      break;
  }
  return (ok);
}

bool testPos()
{     // trouve la position définie dans la commande
  bool ok = false;
  byte pos = tamponRecep[3] - '0';
  if (Acteurs[actionneur].nbPos >= pos)  // si cet actionneur dispose de ce numéro de position
  {
    ok = true;
    if (DEBUGUSB)
      Serial.printf(" en code position %d \n", pos);
  }
  return (ok);
}

bool testCrc()
{     // retourne vrai si le tampon reçu a un crc qui correspond
  byte crc = 0;
  for (char num = 1; num < numChar - 2; num++) // on explore les caractères utiles
    crc += tamponRecep[num];      // on ajoute tous les caractères
  crc &= 0xFF;                    // on écrète à 1 octet
  if (DEBUGCRC)                   // afficher le crc recu, le crc calculé 
    Serial.printf ("crc recu : %d | calculé : %d \n", crc, tamponRecep[numChar-1]);
  return (crc == tamponRecep[numChar-1]);   // on renvoie le résultat de la comparaison
}

bool testCde()
{     // reconnait le type d'ordre, retournfaux sinon
  bool ok = false;
  switch (tamponRecep[1])
  {
    case 'P':      // si on a demandé une mise en position
      if (DEBUGUSB) // afficher ce que l'on demande
        Serial.print ("Ordre de Mise en Position de ");
      ok = true;
      typeOrdre = Pilotage;
      break;
    case 'E':      // si on a demandé un état courant
      if (DEBUGUSB) // afficher ce que l'on demande
        Serial.print ("Ordre de Requete de l'état de ");
      ok = true;
      typeOrdre = DemandeEtat;
      break;
    case 'D' :    // on pilote les moteurs embase
      if (DEBUGUSB) // afficher ce que l'on demande
        Serial.print ("Ordre de Pilotage Embase ");
      ok = true;
      typeOrdre = Deplace;
      break;
    case 'S' :    // on demande l'état du capteur de position / déplacement
      if (DEBUGUSB) // afficher ce que l'on demande
        Serial.print ("Ordre questionnement position/déplacement");
      ok = true;
      typeOrdre = OTOS;
      break;
    case 'C' :    // on questionne les capteurs logique / de distance
      if (DEBUGUSB) // afficher ce que l'on demande
        Serial.print ("Ordre questionnement capteurs de distance et logiques");
      ok = true;
      typeOrdre = Capteurs;
      break;
    case 'M' :      // on actionne ou arrete une séquence
      if (DEBUGUSB) // afficher ce que l'on demande
        Serial.print ("Ordre questionnement capteurs de distance et logiques");
      ok = true;
      typeOrdre = Sequence;
      break;
  }
  return (ok);
}

bool testReception()
{       // cette fonction teste si les commandes sont bien formées
  bool ok = false;          // si tput est bon, il deviendra vrai
  if (testCrc())            // teste le CRC (en position 4)
  {
    if (testCde())          // teste code commande (en position 1)
    {
      switch (typeOrdre)
      {
        case Pilotage :
          if (testOrgane())     // teste code organe (en position 2)
          {
            if (testPos())    // teste code position (en position 3)
              ok = true;
          }
          break;
        case DemandeEtat :
          if (testOrgane())     // teste code organe (en position 2)
          {
            if (tamponRecep[3] == ' ')
              ok = true;
          }
          break;
        case OTOS :
          if (valN(3, 3))     // si ce qui suit est un nombre sur 3 caractères
          {
            rythmeOtos = enVal(3, 3); // ce nombre devient le rythme des envois Otos.
            ok = true;
          }
          break;
        case Capteurs :     // pour les capteurs, pas de paramètres
          ok = true;
          break;
        case Deplace :
          if (testMouvement())
            ok = true;
          break;
        case Sequence :
          if (testSequence())
            ok = true;
          break;
      }
    }
  }
  if (!ok)                  // j'ai pas tout compris, je le dit
  {
    Serial.write (NAK);     // si non, envoie NAK
    ordreRecu = false;
  }
  else
    Serial.write (ACK);
  return (ok);
}

void traiteReception()
{   // la trame reçue est compréhensible, il faut la traiter !
  switch (typeOrdre)
  {
    case Pilotage:   // si c'est une commande de pilotage
      pilote();
      break;
    case DemandeEtat :
      questionne();     // questionne l'actionneur et renvoi sa position
      break;
    case Capteurs :
      interroge();        // lit les capteurs et envoi leur état
      break;
    case Deplace :
      bougeEnVitesse();   // envoi aux moteurs les consignes de vitesse
      break;
    case OTOS :
      otos();     // programmation des envois des données Otos
      break;
    case Sequence :
      lanceSequence(sequence);
      break;
  }
  ordreRecu = false;    // on a fait ce qu'il fallait, on a fini de traiter l'ordre
}

/**************   FONCTIONS DE TEST DES TRAITEMENTS DES ORDRES   ****************/
byte calculCRC (String chaine)
{
  uint16_t crc = 0;
  if (DEBUGCRC)     Serial.print("Calcul CRC : ");
  for (byte c = 1; chaine[c] != ETX; c++)
  {
    crc += chaine[c];
    if (DEBUGCRC)
      Serial.printf ("lettre : %c | valeur %d | crc %d\n", chaine[c], chaine[c], crc);
  }
  crc &= 0xFF;   crc |= 0x80;
  if (DEBUGCRC)   Serial.printf ("crc calculé = %d ",crc);
  return (crc);
}

void testReceptionOrdre()
{    // créé une trame dans le tampon de réception et vérifie qu'elle est bien traitée
  char tamponRecep[6] = {STX, 'P', '2', '1', ' ', ETX};
  tamponRecep[4] = calculCRC("P21");
  // char tamponRecep[5] = {STX, 'E', '2', ' ', ETX};
  // tamponRecep[3] = calculCRC("E2");
  // char tamponRecep[6] = {STX, 'P', 'B', '1', ' ', ETX};
  // tamponRecep[4] = calculCRC("PB1");
  ordreRecu = true;
  if (testReception())
    traiteReception();      // exécution des commandes
}

void testReceptionSequence()
{
  char tamponRecep[6] = {STX, 'M', 'P',' ', ' ', ETX};
  tamponRecep[4] = calculCRC("MP ");
  ordreRecu = true;
  if (testReception())
    traiteReception();      // exécution des commandes
}
#endif