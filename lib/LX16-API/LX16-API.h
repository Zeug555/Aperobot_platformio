#ifndef LX16_API_H
#define LX16_API_H

/*********************************************************************
* Cette bibliothèque est une API issue de la documentation HIWONDER
* élaborée pour valider l'emploi de servomoteurs série de type LX 16A
* pour le prochain robot 2025 du club robotique CROF du lycée Fourcade.
**********************************************************************
* On utilise la carte BusLX16-RS485 et un port série RX-Tx d'un
* microcontroleur, et in signal de sens de dialogue pour la mise au point,
* une Nucléo 64 : la NucléoL476RG, prévu aussi pour Nucléo 32
**********************************************************************/
#include <Arduino.h>

//*** MACROS DE MISE EN FORME (octet <--> mot)
#define GET_LOW_BYTE(A) (uint8_t)((A))  // retourne l'octet faible d'un entier 16 bits
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8) // retourne l'octet fort d'un entier 16 bits
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B)) // contruit un entier 16 bits
// avec les paramètres poids fort, poids faible

// Liste des valeurs et des fonctions exploitées dans les trames
#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

// liste des servomoteurs présents avec leur identificateur
#define ID0   2
#define ID1   3
#define ID2   4
#define ID3   1       // pour tester un servomoteur neuf

// liste des modes d'emploi des servomoteur
#define modeServo   0   // lorsque le LX16 est un servomoteur
#define modeMoteur  1   // lorsqu'il est en moteur DC



/*************    FONCTIONS DE LA BIBLIOTHEQUE   *******************/


byte LobotCheckSum(byte buf[]) 
{   // Cette fonction calcule et retourne le BCC au sein d'un tampon (la taille est indiquée en case 3)
  byte i;             // DANGER i sert ici de compteur dans la boucle et de résultat du BCC
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++)  // on commence après l'entête   
  {
    temp += buf[i];   // le BCC est le compément de la somme 
  }
  temp = ~temp;       // complémentation bit à bit
  return (byte)temp;  // on n'envoie que l'octet faible du résultat
}

void ecritTampon(byte buf[], byte taille)
{
  for (int i = 0; i < taille; i++)
  {
    Serial.print(buf[i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");          // saut de ligne après l'affichage
}

/****************   FONCTIONS   DE   DIALOGUE   LX16    *****************/

void initSens()
{     // selon la configuration des interfaces on pilote les signaux
#if defined(NUCLEO64_LS126)
  pinMode (SENS, OUTPUT);       // SENS est une sortie de pilotage du halfduplex
  pinMode (SENSL, OUTPUT);
#endif
#if defined (NUCLEO32_LS126)
  pinMode (SENS, OUTPUT);       // SENS est une sortie de pilotage du halfduplex
  pinMode (SENSL, OUTPUT);
#endif
#if defined (NUCLEO64_BUSLX16)
  pinMode (SENS2, OUTPUT);
#endif
#if defined (NUCLEO64_LS126)
  pinMode (SENS2, OUTPUT);
#endif
}

void metEmission()
{
   // selon la configuration des interfaces on pilote les signaux
#if defined(NUCLEO64_LS126)
  digitalWrite(SENS, EMET);      // on se place en émission
  digitalWrite (SENSL, LIT);
#endif
#if defined(NUCLEO32_LS126)
  digitalWrite(SENS, EMET);      // on se place en émission
  digitalWrite (SENSL, LIT);
#endif
#if defined(NUCLEO64_BUSLX16)
  digitalWrite(SENS2, EMET);      // on se place en émission
#endif
#if defined(NUCLEO32_BUSLX16)
  digitalWrite(SENS2, EMET);      // on se place en émission
#endif
}

void metReception()
{
     // selon la configuration des interfaces on pilote les signaux
#if defined(NUCLEO64_LS126)
  digitalWrite(SENS, LIT);      // on se place en émission
  digitalWrite (SENSL, EMET);
#endif
#if defined(NUCLEO32_LS126)
  digitalWrite(SENS, LIT);      // on se place en émission
  digitalWrite (SENSL, EMET);
#endif
#if defined(NUCLEO64_BUSLX16)
  digitalWrite(SENS2, LIT);      // on se place en émission
#endif
#if defined(NUCLEO32_BUSLX16)
  digitalWrite(SENS2, LIT);      // on se place en émission
#endif
}

void envoi(byte buf[], byte taille)
{
  metEmission();
  delayMicroseconds(20);
  for (int i = 0; i < taille; i++)
  {
    BusLX16.write(buf[i]);       // sur Nucléo 64
    //BusPAP.write(buf[i]);     // sur Nucléo 32
  }
  BusLX16.flush();               // on attend que tout le tampon soit envoyé
  delayMicroseconds(30);         // petite attente avant retournement du MAX487
  metReception();                // on repasse en mode réception sur le LS126
}

void LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time)
{   // Cette fonction fait tourner à la position voulu en absolu en un temps donné
  byte taille = 10;
  byte buf[taille];
  // limitation des valeurs extrèmes de la position 0 .. 1000
  if (position < 0)
    position = 0;
  if (position > 1000)
    position = 1000;
  // Construction de la trame de déplacement radial
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO MOVE TO POSITION : ");  // affichage de la fonction
    ecritTampon(buf, taille);                         // et de la trame
  }
  envoi (buf, taille);                // envoi du tampon sur le bus
}

void LobotSerialServoStopMove(uint8_t id)
{   // Cette fonction arrête la rotation du servomoteur
  // construction du tampon à émettre
  byte taille = 6;
  byte buf[taille];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO STOP : ");    // affichage de la fonction
    ecritTampon(buf, taille);               // et de la trame
  }
  envoi (buf, taille);                // envoi du tampon sur le bus
}

void LobotSerialServoStartMove(uint8_t id)
{   // Cette fonction Démarre la rotation du servomoteur
  // construction du tampon à émettre
  byte taille = 6;
  byte buf[taille];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_START;
  buf[5] = LobotCheckSum(buf);
  if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO START : ");     // affichage de la fonction
    ecritTampon(buf, taille);                 // et de la trame
  }
  envoi (buf, taille);                // envoi du tampon sur le bus
}

void LobotSerialServoSetID(uint8_t oldID, uint8_t newID)
{   // Cette fonction change le numéro d'identification du servomoteur
  // Construction du tampon à émettre
  byte taille = 7;
  byte buf[taille];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO ID WRITE : ");  // affichage de la fonction
    ecritTampon(buf, taille);                 // et de la trame
  }
  envoi (buf, taille);                // envoi du tampon sur le bus
}

void LobotSerialServoSetMode(uint8_t id, uint8_t Mode, int16_t Speed)
{   // Cette fonction fixe le mode du servomoteur : continu/angulaire et la vitesse
  byte taille = 10;
  byte buf[taille];
  // Construction du tampon à émettre
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);
    if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO SET MODE : ");  // affichage de la fonction
    ecritTampon(buf, taille);                 // et de la trame
  }
  envoi (buf, taille);                // envoi du tampon sur le bus
}

void LobotSerialServoLoad(uint8_t id)
{   // Cette fonction met sous tension le servomoteur
  // construction du tampon à émettre
  byte taille = 7;
  byte buf[taille];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO LOAD WRITE : ");  // affichage de la fonction
    ecritTampon(buf, taille);                   // et de la trame
  }
  envoi (buf, taille);                // envoi du tampon sur le bus
}

void LobotSerialServoUnload(uint8_t id)
{   // Cette fonction débraye le servomoteur de l'alimentation
  // Construction du tampon à émettre
  byte taille = 7;
  byte buf[taille];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
    if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO UNLOAD WRITE : ");  // affichage de la fonction
    ecritTampon(buf, taille);                     // et de la trame
  }
  envoi (buf, taille);                // envoi du tampon sur le bus
}

int LobotSerialLitTrame (byte *reponse)
{   // cette fonction attend sur la liaison série la réception d'une trame
// rappel, les trames sont 0x55 0x55 'moteur' 'longueur' 'instruction' 'valeurs....' 'checksum'
  bool trameComencee = false;   // on a reçu un début de trame
  bool receptionFinie = false;  // on a recu la fin de trame
  byte cptrRecu = 0;      // compteur des octets reçus
  byte cptrDonnee = 0;    // compteur des octets utiles
  byte longueur = 2;      // nombre d'octet à recevoir   !!! PAS TOUJOURS !!!
  byte recu;              // octet lu dans la réception
  byte taille = 32;       // taille du tampon de réception
  byte tampon[taille];    // tampon pour stocker la réception d'une trame
  byte i;

  if (DEBUGLITSM) 
    Serial.print ("*");            // témoin de passage dans la fonction
  while (BusLX16.available())      // tant que le servo répond
  {
    recu = BusLX16.read();         // lecture du nouvel octet
    delayMicroseconds(200);        // INDISPENSABLE !!!
    if (DEBUGLITSM) 
    {  Serial.print(recu, HEX); Serial.print("-");   }  // affichage de l'octet reçu

    if (!trameComencee)             // si la trame n'a pas encore commencée
    {
      if (recu == 0) continue;      // si recu 0 : erreur de break, on s'en moque
      if (recu == LOBOT_SERVO_FRAME_HEADER)   // Le second caractère est-il 0x55 ?
      {
        tampon[cptrRecu] = recu;  // on range le 0x55
        cptrRecu++;               // comptage de ce caractère
        if (cptrRecu == 2)        // si je viens de recevoir le second 0x55 ?
        {
          cptrRecu = 0;           // on efface le compteur d'entête
          trameComencee = true;   // on enregistre qu'une nouvelle trame a commencé   
          cptrDonnee = 1;         // on remplit le tampon à partir de 1 pour que l'ID mot soit en case 2
        }
      }
      else                        // si on a recu autre chose que 0x55 pour commencer
      {
        cptrRecu = 0;             // on annule la réception en cours
        return -3;                // on retourne -3 pour signaler "erreur de début de trame"
      }
    }

    if (trameComencee)            // si une trame a commencé
    {     // le dernier octet reçu est le second 0x55
      tampon[cptrDonnee] = (uint8_t)recu;   // on range l'octet reçu dans le tampon
      
      if (DEBUGSM)          // affichage de la trame en cours de constitution
      {
        for (byte i=0; i < cptrDonnee; i++)
        {
          Serial.print(tampon[i], HEX); Serial.print("|");
        }
        Serial.println();
      }
      
      if (cptrDonnee == 3)        // si c'est le quatrième octet, c'est la longueur
      {
        longueur = tampon[cptrDonnee];  // on fixe la longueur attendue de la trame
        if (longueur < 3)               // si la trame a une taille improbable
        {
          trameComencee = false;        // on recommence la réception à 0
          return -2;                    // on retourne -2 pour signaler "erreur de taille"
        }
      }
      cptrDonnee++;                     // on passe à l'octet suivant
      if (cptrDonnee > 8)               // si la trame a une taille improbable
      {
        trameComencee = false;        // on recommence la réception à 0
        return -2;                    // on retourne -2 pour signaler "erreur de taille"
      }
      if (cptrDonnee == longueur + 3)   // si on a rangé le checksum
      {
        if (DEBUGLITSM)                    // témoin de l'affichage de la trame reçue
        {
          Serial.print("DONNEES RECUES : ");
          for (i = 0; i < cptrDonnee; i++) // on va explorer le tampon
          {
            Serial.print(tampon[i], HEX);
            Serial.print("_");
          }
          Serial.println(" ");
        }
        // appel du checksum avec le dernier octet rangé
        if (LobotCheckSum(tampon) == tampon[cptrDonnee - 1]) // qui est le checksum recu
        {
          if (DEBUGLITSM)
            Serial.println("CheckSum OK!!");
          trameComencee = false;
          memcpy(reponse, tampon + 4, longueur - 2);  // on retourne un tampon qui contient CMD et PARAMs
          return 1;                           // on retourne 1; > 0 pour signaler "tout est OK"
        }
        return -1;                            // on retourne -1 pour indiquer "erreur de checksum"
      }         // fin de compteur == longueur + 3 ?
    }           // fin de trameCommencee
  }             // fin du while BusLX16.available()
  return -3; // Bus pas disponible 
}

int gereReponse(byte *buf, byte nbOctets)
{   // on attend la réception de la trame, on la décode on retourne erreur ou valeur sur nbOctets
  int16_t reponse;
  reponse = LobotSerialLitTrame(buf);     // lecture de la trame réponse
  if (reponse > 0)                    // si réponse > 0, lecture OK
  {
    if (nbOctets == 2)                // selon le nombre d'octets utiles attendus
      reponse = (int16_t)BYTE_TO_HW(buf[2], buf[1]);      // construction de la position servo et Vin
    if (nbOctets == 1)
      reponse = (int16_t)buf[1];      // construction de la température
  }
  else                                // si réponse < 0, il y a eu problème de lecture
  {
    if (DEBUGLITSM)                   // si on veut les afficher
    {
      if (reponse == -1)    Serial.println ("Erreur de checksum");
      if (reponse == -2)    Serial.println ("Erreur taille de trame");
      if (reponse == -3)    Serial.println ("Erreur de début de trame");
      if (reponse == -4)    Serial.println ("5 essais non concluants");
      if (reponse == -2048) Serial.println ("Erreur de TimeOut");
    }
  }
  return reponse;
}

int LobotSerialServoReadPosition(uint8_t id)
{   // cette fonction questionne la position courante du servomoteur
  int count = 10000;          // soit environ 2200 us
  int reponse;
  byte etatReponse;
  byte taille = 6;
  byte buf[6];
  unsigned long debutEcoute, finEcoute;

  // construction de la trame de requète
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);
  if (DEBUGLITSM)
  {
    Serial.print("LOBOT SERVO READ POSITION : ");   // affichage de la fonction
    ecritTampon(buf, taille);                       // et de la trame
  }
  // on est déjà en mode réception

  envoi(buf, taille);                // envoi du tampon sur le bus avec recopie console

  while (BusLX16.available())                   // vidange du tampon de réception
  {
    char c = BusLX16.read();                     
    if (DEBUGLITSM)
    {  Serial.print(c, HEX); Serial.print("/"); } // affichage du reliquat du tampon réponse précédente
  }
  if (DEBUGLITSM)     Serial.print("\n");

  // on est revenu en mode réception, le servo répond 170us après la fin de la trame d'appel
  debutEcoute = micros();
  while (!BusLX16.available()) {     // attente qu'il y ait un début de réponse
    count -= 1;
    if (count < 0)
    {
      if (DEBUGLITSM)      Serial.println ("#");
      finEcoute = micros();
      if (DEBUGLITSM)
        Serial.printf("Durée de l'attente précédente %d\n", finEcoute - debutEcoute);
      return -2048;                   // retour "TimeOut"
    }
  }
  // arrivé ici, on a des octets présents dans la réponse
  reponse = gereReponse(buf, 2);    // on attend et on décode on retourne erreur ou valeur sur 2 octets
  if (DEBUGLITSM)
  { Serial.printf ("SM %d en Position %d\n ", id, reponse); 
  }
  return reponse;
}

int LobotSerialServoReadVin(uint8_t id)
{   // cette fonction questionne la tension actuelle appliquée au servomoteur
  int count = 10000;
  int reponse;
  byte taille = 6;
  byte buf[taille];
  // construction de la trame de requete
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);
    if (DEBUGSM)
  {
    Serial.print("LOBOT SERVO VIN READ : ");  // affichage de la fonction
    ecritTampon(buf, taille);                 // et de la trame
  }
  envoi(buf, taille);
  // compilation de la réponse (2 octets utiles sur 6 ici)
  while (BusLX16.available())
    BusLX16.read();
  BusLX16.write(buf, 6);                // envoi du tampon sur le bus
  while (!BusLX16.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }
  if (LobotSerialLitTrame(buf) > 0)
    reponse = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    reponse = -2049;
  if (DEBUGSM)
    Serial.println(reponse);
  return reponse;
}

int repeteLecture (byte moteur, int ordreEnvoi, byte nbFois)
{     // cette fonction envoie un ordre qui nécessite une réponse jusqu'à nbfois
  // on répète si la réponse présente une erreur. On renvoie la valeur si c'est correct
  int16_t reponse;
  byte ess = 0;
  if (DEBUGLITSM)     Serial.printf ("REPETE %d FOIS : ", nbFois);
  while (ess < nbFois)
  {
    switch (ordreEnvoi)           // on étudie la nature de la demande
    {
      case LOBOT_SERVO_POS_READ : // si c'est une demande de position
          reponse = LobotSerialServoReadPosition(moteur);  // je demande et on a une réponse
          if (reponse >= 0) 
            return (reponse); 
          else 
            Serial.printf("retour négatif de %d\n", reponse);
          break;
      case LOBOT_SERVO_VIN_READ :
          reponse = LobotSerialServoReadVin(moteur);
          if (reponse >= 0) 
            return (reponse); 
          break;
    }
    // si on arrive là, c'est qu'il y a eu un problème, on recommence
    ess ++;
  }  // on a tout essayer ... ça marche pas
  if (DEBUGLITSM)
    Serial.printf ("5 essais, non concluants de %d \n", ordreEnvoi); // on affiche 
  return (-4);// et on retourne que l'on ne recoit pas de réponse correcte
}

#endif
