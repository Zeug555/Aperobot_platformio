/*******************************************************
* Fichier : servo42d.h                                *
* Description : Déclarations d'énumérations,          *
* prototypes de fonctions et table des codes RS485    *
* liés aux commandes des drivers MKS SERVO42D         *
********************************************************/

#ifndef SERVO42D_H
#define SERVO42D_H

#include <stdint.h>  // Types comme uint8_t, uint16_t, etc.
#include <vector>    // std::vector
#include <Arduino.h> // Pour String et fonctions Arduino
#include "global.h"  // Définitions globales comme nb_mot ou E_R

// ! Constantes globales
// ? Utilisées pour le formatage et la configuration des moteurs
constexpr size_t TAILLE_TRAME = 11;                 // Taille max d'une trame RS485
constexpr uint8_t ACCELERATION_PAR_DEFAUT = 0;      // Accélération par défaut si non spécifiée
constexpr bool REPONSE_AUTORISEE = true;            // Indique si le moteur doit répondre après commande

#define DESCEND   1          // valeur du sens ascenseur pour descendre
#define TRIG_HOME 1          // valeur du signal sur l'ascenseur quand le 'HOME' est actif (appuyé)

// ! Enumération des commandes logiques
// ? Chaque commande correspond à un index, mappé à un vrai code RS485 via CODE_TRAME
enum class Commande : uint8_t {
    Calibration = 0,           // 0
    MouvementRelatif,          // 1
    MouvementVitesse,          // 2
    ModeFonctionnement,        // 3
    Intensite,                 // 4
    PourcentageIntensite,      // 5
    MicroPas,                  // 6
    Direction,                 // 7
    EteindreEcran,             // 8
    ChangerAdresse,            // 9
    Stop,                      // 10
    Activer,                   // 11
    DemanderPosition,          // 12
    DemanderVitesse,           // 13
    DemanderEnable,            // 14
    Reponse,                   // 15
    LectureConfiguration,      // 16
    SetProtect,
    SetMplyer,
    SetBaudrate,
    GoHome,
    SetHomeParams,
    SetHomeZero,
    RestoreDefaults,
    DemanderEtatHome
};

// ! Table des vrais codes de trame
// ? Associée directement aux valeurs de l'enum Commande
constexpr uint8_t CODE_TRAME[] = {
    0x80, // Calibration
    0xFD, // MouvementRelatif
    0xF6, // MouvementVitesse
    0x82, // ModeFonctionnement
    0x83, // Intensite
    0x9B, // PourcentageIntensite
    0x84, // MicroPas
    0x86, // Direction
    0x87, // EteindreEcran
    0x8B, // ChangerAdresse
    0xF7, // Stop
    0x85, // Activer
    0x33, // DemanderPosition
    0x32, // DemanderVitesse
    0x3A, // DemanderEnable
    0x8C, // Reponse
    0x47, // lecture config
    0x88, // SetProtect
    0x89, //SetMplyer
    0x8A, //SetBaudrate
    0x91, //GoHome
    0x90, //SetHomeParams
    0x92, //SetHomeZero
    0x3F, //RestoreDefaults
    0x3B  //demander_etat_home
};

// ! Structure moteur
// ? Contient toutes les infos pour piloter et suivre un moteur individuel
struct Moteur {
    uint8_t id;                                 // Adresse RS485 du moteur
    String libelle;                             // Nom pour debug ou affichage
    Commande commande_en_cours;                // Dernière commande envoyée
    uint32_t position_absolue;                 // Position actuelle connue
    uint16_t vitesse;                          // Vitesse configurée
    bool inversion;                            // Inverse la direction si true
    uint8_t buffer_tx[TAILLE_TRAME] = {};      // Buffer d'émission
    uint8_t buffer_rx[TAILLE_TRAME] = {};      // Buffer de réception
    uint32_t pulsions;                         // Nombre d'impulsions à effectuer (si relatif)
    uint8_t sens;                              // 0 = horaire, 1 = antihoraire
    uint8_t acceleration = ACCELERATION_PAR_DEFAUT; // Accélération appliquée
    bool doit_repondre = REPONSE_AUTORISEE;         // Active ou non l'attente de réponse
};

// ! Tableau global des moteurs
// ? Accessible dans tout le programme
extern Moteur mot[nb_mot];

// ! Envoie une commande générique
// * envoyer_commande(mot[0], Commande::Stop, {0});
void envoyer_commande(Moteur& moteur, Commande commande, const std::vector<uint8_t>& donnees);

// ! Envoie une commande de type vitesse
// * envoyer_vitesse(mot[0], 0, 150, 10);
void envoyer_vitesse(Moteur& moteur, uint8_t sens, uint16_t vitesse, uint8_t acceleration);

// ! Fonction de réception de trame avec vérification du checksum
// * recevoir_reponse(mot[0], reponse);
bool recevoir_reponse(Moteur& moteur, std::vector<uint8_t>& reponse);

// ! Initialisation du tableau de moteurs
// * init_moteurs();
void init_moteurs();

// ! Mise à jour cyclique des moteurs
// * Appelée régulièrement pour logique de suivi ou polling
void mettre_a_jour_moteurs();

// ! Interprétation des messages RS485 reçus
// * Utilisé pour mode esclave ou analyse passive
void recevoir_et_interpreter_message_RS485();

// ! Mouvement relatif par impulsions
// * MouvRelPulse(0, 1, 150, 10, 10000);
void MouvRelPulse(uint8_t adr_moteur, uint8_t direction, uint16_t vitesse, uint8_t acceleration, uint32_t impulsions);

bool questionneHome(int8_t moteur_index, uint16_t timeout_ms, uint16_t intervalle_poll_ms);

// ! Arrêt d’un moteur ou de tous les moteurs (broadcast)
// * arret_moteur(0, 10); ou arret_moteur(nb_mot, 10);
void arret_moteur(int8_t adr_moteur);

// ! Envoie la commande de calibration de l'encodeur au moteur
// * Peut être utilisé avec mot_tous = -1 pour le broadcast
void calibrer_encodeur(int8_t moteur_index);

// ! Définit le courant du moteur (mA), maximum recommandé = 3000 (pour SERVO42D)
// * Peut être utilisé avec mot_tous = -1 pour le broadcast
void set_courant(int8_t moteur_index, uint16_t courant_mA);

// ! Définit la subdivision du moteur (microstep) : valeur 0x00 à 0xFF
void set_subdivision(int8_t moteur_index, uint8_t microstep);

// ! Définit le mode de la broche En (Enable) :
// * 0x00 = actif bas, 0x01 = actif haut, 0x02 = toujours actif
void set_enable_pin_mode(int8_t moteur_index, uint8_t mode_en);

// ! Définit la direction par défaut du moteur : 0 = CW, 1 = CCW
void set_direction(int8_t moteur_index, uint8_t direction);

// ! Active ou désactive l'extinction automatique de l'écran
// * enable = 0 (désactivé), 1 (activé)
void set_auto_screen_off(int8_t moteur_index, uint8_t enable);

// ! Active ou désactive la protection contre les moteurs bloqués
// * enable = 0 (désactivé), 1 (activé)
void set_locked_rotor_protection(int8_t moteur_index, uint8_t enable);

// ! Active ou désactive la fonction d’interpolation de microsteps
// * enable = 0 (désactivé), 1 (activé)
void set_interpolation(int8_t moteur_index, uint8_t enable);

// ! Définit le baudrate (voir tableau de correspondance dans la doc)
// * baud = 1 (9600), 2 (19200), ..., 7 (256000)
void set_baudrate(int8_t moteur_index, uint8_t baud_code);

// ! Définit une nouvelle adresse pour le moteur (0 à 255)
// * ⚠️ Nécessite de mettre à jour l’objet mot[] après le changement !
void set_slave_address(int8_t moteur_index, uint8_t new_address);

// ! Active ou désactive la réponse du moteur aux trames (F1...)
// * enable = 1 : répond (par défaut), 0 : ne répond plus
void set_slave_respond(int8_t moteur_index, uint8_t enable);

// ! Lance le retour à l’origine (GoHome)
void go_home(int8_t moteur_index);

// ! Configure le comportement du retour à l’origine (home)
// * trig = 0 (low) ou 1 (high), dir = 0 (CW) ou 1 (CCW), speed = 0–3000 (RPM)
void set_home_parameters(int8_t moteur_index, uint8_t trig, uint8_t dir, uint16_t speed);

// ! Restaure les paramètres usine du moteur (attention, irréversible !)
void restore_factory_settings(int8_t moteur_index);

// lit une réponse à une commande d'un stepper
bool lire_reponse_variable(uint8_t code_attendu, std::vector<uint8_t>& data_out, uint8_t taille_donnee_attendue, uint8_t id_attendu);

#endif
