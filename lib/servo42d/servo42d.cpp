/*******************************************************
* Fichier : servo42d.cpp                              *
* Description : Fonctions de communication RS485      *
* avec les drivers MKS SERVO42D : vitesse, position,  *
* configuration, reset, calibration, etc.             *
* Auteur original : Fourcade CROF (Willace & JP)      *
* Refactorisé par : Valentin Bonali - 2025            *
********************************************************/

#include "servo42d.h"
#include "WSerial.h"

Moteur mot[nb_mot]; 

// ! Init moteurs
// ? Initialise le tableau de moteurs avec des valeurs de départ.
// * À appeler dans setup()
// TODO: Ajouter gestion dynamique si nécessaire
void init_moteurs() {
    for (int i = 0; i < nb_mot; ++i) {
        mot[i].id = i + 1;  // Attribue un ID unique
        mot[i].libelle = "Moteur" + String(i + 1);  // Nom d'affichage
        mot[i].commande_en_cours = Commande::Stop;  // État initial : arrêté
        mot[i].position_absolue = 0;  // Position connue = 0
        mot[i].vitesse = 0;  // Vitesse par défaut
        mot[i].inversion = false;  // Pas d'inversion du sens
        mot[i].pulsions = 0;  // Zéro mouvement en attente
        mot[i].sens = 0;  // Sens horaire par défaut
        mot[i].acceleration = ACCELERATION_PAR_DEFAUT;  // Valeur d'accélération standard
        mot[i].doit_repondre = REPONSE_AUTORISEE;  // Le moteur répond aux commandes
    }
}

// ! Envoie d'une commande générique RS485
// ? Génère une trame pour le moteur donné, et l'envoie via BusPAP en RS485
// * À utiliser pour toute commande personnalisée ou avancée
void envoyer_commande(Moteur& moteur, Commande commande, const std::vector<uint8_t>& donnees) {
    std::vector<uint8_t> trame;

    // Construction de la trame
    trame.push_back(0xFA); // Header
    trame.push_back(moteur.id); // Adresse
    trame.push_back(CODE_TRAME[static_cast<uint8_t>(commande)]); // Code

    // Ajout des données
    trame.insert(trame.end(), donnees.begin(), donnees.end());

    // CRC
    uint8_t crc = 0;
    for (uint8_t b : trame) crc += b;
    crc &= 0xFF;
    trame.push_back(crc);

    // DEBUG
    if (DEBUG_AFF_TR) {
        Serial.print("[TX] ");
        for (uint8_t b : trame) {
            Serial.print(b < 16 ? "0" : "");
            Serial.print(b, HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Envoi réel
    digitalWrite(E_R, HIGH);
    for (uint8_t b : trame) {
        BusPAP.write(b);
    }
    BusPAP.flush();            // Assure l'envoi complet
    delayMicroseconds(150);    // Petit délai pour éviter collision
    digitalWrite(E_R, LOW);    // Retour en réception
}

// ! Fonction globale de lecture des réponses du moteur
// * Format attendu : FB [id] [code] [status] [CRC]
// ? Vérifie que le code reçu correspond à celui attendu et que le CRC est valide
// ? Peut retourner le statut brut via un pointeur (status_out)
// TODO: Améliorer la gestion du timeout si besoin
// @param code_attendu Le code de commande attendu (ex : 0x80 pour calibration)
// @param status_out (optionnel) pointeur pour recevoir le statut brut (0, 1, 2...)
// @return true si le status == 1 (succès), false sinon
//
// Exemple d'utilisation :
//
//   calibrer_encodeur(mot_droit);
//   delay(200);
//   if (recevoir_reponse(0x80)) {
//       Serial.println("Calibration OK");
//   } else {
//       Serial.println("Calibration échouée");
//   }

bool recevoir_reponse(uint8_t code_attendu, int8_t* status_out) {
    digitalWrite(E_R, LOW);  // Mise en réception

    unsigned long timeout = millis() + 500;

    while (!BusPAP.available()) {
        if (millis() > timeout) {
            Serial.println("[ERREUR] Timeout en attente de réponse.");
            return false;
        }
    }

    if (BusPAP.read() != 0xFB) {
        Serial.println("[ERREUR] Mauvais header dans la réponse !");
        return false;
    }

    uint8_t id = BusPAP.read();
    uint8_t code = BusPAP.read();
    // maintenant on va gérer la taille de status = données de réponse
    uint8_t status = BusPAP.read();
    uint8_t crc_recu = BusPAP.read();

    uint8_t crc_calcule = (0xFB + id + code + status) & 0xFF;

    if (crc_recu != crc_calcule) {
        Serial.print("[ERREUR] CRC invalide ! Calculé : ");
        Serial.print(crc_calcule, HEX);
        Serial.print(" / Reçu : ");
        Serial.println(crc_recu, HEX);
        return false;
    }

    if (code != code_attendu) {
        Serial.print("[ERREUR] Code réponse inattendu : 0x");
        Serial.println(code, HEX);
        return false;
    }

    if (status_out != nullptr) {
        *status_out = status;
    }

    if (DEBUG_AFF_REP) {
        Serial.print("[REPONSE] Code 0x");
        Serial.print(code, HEX);
        Serial.print(" → Status : ");
        Serial.println(status);
    }

    return (status == 1);
}

// Lit_reponse_variable() Permet de lire une valeur demandée à un moteur pas à pas.
// le paramètre important est le nombre d'octets utiles de données collectées
// le résultat sera transmis dans le vecteur data_out à ranger
bool lire_reponse_variable(uint8_t code_attendu, std::vector<uint8_t>& data_out, uint8_t taille_donnee_attendue, uint8_t id_attendu) {
    digitalWrite(E_R, LOW);  // Réception
    unsigned long timeout = millis() + 500;

    while (BusPAP.available() < (3 + taille_donnee_attendue + 1)) {
        if (millis() > timeout) {
            Serial.println("[Timeout] Pas de réponse");
            return false;
        }
    }

    if (BusPAP.read() != 0xFB) return false;

    uint8_t id = BusPAP.read();
    if (id != id_attendu) {
        Serial.print("[Erreur] Mauvais ID de réponse : ");
        Serial.println(id);
        return false;
    }

    uint8_t code = BusPAP.read();
    if (code != code_attendu) {
        Serial.print("[Erreur] Code inattendu : ");
        Serial.println(code, HEX);
        return false;
    }

    data_out.clear();
    uint8_t crc_calc = 0xFB + id + code;

    for (uint8_t i = 0; i < taille_donnee_attendue; ++i) {
        uint8_t val = BusPAP.read();
        data_out.push_back(val);
        crc_calc += val;
    }

    uint8_t crc_recu = BusPAP.read();
    if ((crc_calc & 0xFF) != crc_recu) {
        Serial.println("[Erreur] CRC invalide");
        return false;
    }

    return true;
}

/*****************     FONCTIONS   DE   PILOTAGE    ******************/

// ! Envoie une commande de vitesse au moteur (mode F6 spécifique Servo42D)
// * Exemple : envoyer_vitesse(mot[0], 0, 320, 2);
void envoyer_vitesse(Moteur& moteur, uint8_t sens, uint16_t vitesse, uint8_t acceleration) {
    // Sécurité : la vitesse ne doit pas dépasser 3000, et on garde 12 bits
    uint16_t vitesse_lim = constrain(vitesse, 0, 3000) & 0x0FFF;

    // Byte 4 : sens (bit 7) + bits poids forts de la vitesse (4 bits)
    uint8_t byte4 = (sens << 7) | ((vitesse_lim >> 4) & 0x0F);

    // Byte 5 : bits poids faibles de la vitesse
    uint8_t byte5 = vitesse_lim & 0xFF;

    std::vector<uint8_t> donnees = {
        byte4,
        byte5,
        acceleration
    };

    envoyer_commande(moteur, Commande::MouvementVitesse, donnees);
}

// ! Mouvement relatif (par impulsions)
// ? Fait tourner le moteur d’un certain nombre d’impulsions
// * Exemple : MouvRelPulse(0, 1, 150, 10, 10000);
void MouvRelPulse(uint8_t adr_moteur, uint8_t sens, uint16_t vitesse, uint8_t acceleration, uint32_t impulsions) {
    Moteur& moteur = mot[adr_moteur];
    uint16_t vitesse_lim = constrain(vitesse, 0, 3000) & 0x0FFF;
    uint8_t byte4 = (sens << 7) | ((vitesse_lim >> 4) & 0x0F);
    uint8_t byte5 = vitesse_lim & 0xFF;
    std::vector<uint8_t> donnees = {
        byte4,
        byte5,
        acceleration,
        static_cast<uint8_t>(impulsions & 0xFF),
        static_cast<uint8_t>((impulsions >> 8) & 0xFF),
        static_cast<uint8_t>((impulsions >> 16) & 0xFF),
        static_cast<uint8_t>((impulsions >> 24) & 0xFF)
    };
    envoyer_commande(moteur, Commande::MouvementRelatif, donnees);
}

// ! Envoie une commande d'arrêt d'urgence (F7) au moteur ou en broadcast
// * Exemple : arret_moteur(0);  // Arrêt de tous les moteurs
void arret_moteur(int8_t moteur_index) {
    Moteur moteur;
    if (moteur_index < 0) {
        moteur.id = 0;  // broadcast
    } else {
        moteur = mot[moteur_index];
    }
    std::vector<uint8_t> donnees = {};
    envoyer_commande(moteur, Commande::Stop, donnees);
}

bool questionneHome(int8_t moteur_index, uint16_t timeout_ms = 5000, uint16_t intervalle_poll_ms = 200) {
    Moteur moteur = mot[moteur_index];
    uint32_t t0 = millis();

    while (millis() - t0 < timeout_ms) {
        // Envoie la commande FA ID 3B CRC pour demander l'état du home
        std::vector<uint8_t> reponse;
        envoyer_commande(moteur, Commande::DemanderEnable, {}); // 0x3A pour être en réception active si besoin
        delay(10); // mini délai
        envoyer_commande(moteur, static_cast<Commande>(0x3B), {});  // 0x3B = statut "Go Home"

        if (lire_reponse_variable(0x3B, reponse, 1,moteur.id)) {
            uint8_t status = reponse[0];

            if (status == 1) {
                Serial.println("[HOME] Terminé avec succès !");
                return true;
            } else if (status == 2) {
                Serial.println("[HOME] Échec !");
                return false;
            } else {
                Serial.println("[HOME] En cours...");
            }
        } else {
            Serial.println("[HOME] Pas de réponse valide.");
        }

        delay(intervalle_poll_ms);
    }

    Serial.println("[HOME] Timeout dépassé !");
    return false;
}

// ! Appelé dans loop()
// ? Permet d’ajouter du comportement cyclique comme le polling ou logs
void mettre_a_jour_moteurs() {
    for (int i = 0; i < nb_mot; ++i) {
        // TODO: Ajouter des actions cycliques si besoin
    }
}

// ! Réception de messages RS485
// ? Réagit aux trames entrantes (ex: requêtes du maître ou d’un PC)
void recevoir_et_interpreter_message_RS485() {
    // TODO: Lire et interpréter trames entrantes ici
}

/**********     FONCTIONS DE CONFIGURATIONS   ***********************/

// ! Envoie la commande de calibration de l'encodeur au moteur
// * Peut être utilisé avec mot_tous = -1 pour le broadcast
void calibrer_encodeur(int8_t moteur_index) {
    Moteur moteur;

    // Broadcast ?
    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { 0x00 };  // Trame FA id 80 00 CRC

    envoyer_commande(moteur, Commande::Calibration, donnees);
}

// ! Définit le courant du moteur (mA), maximum recommandé = 3000 (pour SERVO42D)
// * Peut être utilisé avec mot_tous = -1 pour le broadcast
void set_courant(int8_t moteur_index, uint16_t courant_mA) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;  // broadcast
    } else {
        moteur = mot[moteur_index];
    }

    // Courant en little-endian : LSB d’abord
    std::vector<uint8_t> donnees = {
        static_cast<uint8_t>(courant_mA & 0xFF),
        static_cast<uint8_t>((courant_mA >> 8) & 0xFF)
    };

    envoyer_commande(moteur, Commande::Intensite, donnees);
}

// ! Définit la subdivision du moteur (microstep) : valeur 0x00 à 0xFF
void set_subdivision(int8_t moteur_index, uint8_t microstep) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { microstep };

    envoyer_commande(moteur, Commande::MicroPas, donnees);
}

// ! Définit le mode de la broche En (Enable) :
// * 0x00 = actif bas, 0x01 = actif haut, 0x02 = toujours actif
void set_enable_pin_mode(int8_t moteur_index, uint8_t mode_en) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { mode_en };

    envoyer_commande(moteur, Commande::Activer, donnees);
}

// ! Définit la direction par défaut du moteur : 0 = CW, 1 = CCW
void set_direction(int8_t moteur_index, uint8_t direction) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { direction };

    envoyer_commande(moteur, Commande::Direction, donnees);
}

// ! Active ou désactive l'extinction automatique de l'écran
// * enable = 0 (désactivé), 1 (activé)
void set_auto_screen_off(int8_t moteur_index, uint8_t enable) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { enable };

    envoyer_commande(moteur, Commande::EteindreEcran, donnees);
}

// ! Active ou désactive la protection contre les moteurs bloqués
// * enable = 0 (désactivé), 1 (activé)
void set_locked_rotor_protection(int8_t moteur_index, uint8_t enable) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { enable };

    envoyer_commande(moteur, Commande::SetProtect, donnees);
}

// ! Active ou désactive la fonction d’interpolation de microsteps
// * enable = 0 (désactivé), 1 (activé)
void set_interpolation(int8_t moteur_index, uint8_t enable) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { enable };

    envoyer_commande(moteur, Commande::SetMplyer, donnees);
}

// ! Définit le baudrate (voir tableau de correspondance dans la doc)

// * baud = 1 (9600), 2 (19200), ..., 7 (256000)
void set_baudrate(int8_t moteur_index, uint8_t baud_code) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { baud_code };

    envoyer_commande(moteur, Commande::SetBaudrate, donnees);
}

// ! Définit une nouvelle adresse pour le moteur (0 à 255)
// * ⚠️ Nécessite de mettre à jour l’objet mot[] après le changement !
void set_slave_address(int8_t moteur_index, uint8_t new_address) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;  // broadcast (à éviter pour changer d'adresse)
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { new_address };

    envoyer_commande(moteur, Commande::ChangerAdresse, donnees);
}

// ! Active ou désactive la réponse du moteur aux trames (F1...)
// * enable = 1 : répond (par défaut), 0 : ne répond plus
void set_slave_respond(int8_t moteur_index, uint8_t enable) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = { enable };

    envoyer_commande(moteur, Commande::Reponse, donnees);
}

// ! Lance le retour à l’origine (GoHome)
void go_home(int8_t moteur_index) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = {};  // aucun paramètre

    envoyer_commande(moteur, Commande::GoHome, donnees);
}

// ! Configure le comportement du retour à l’origine (home)
// * trig = 0 (low) ou 1 (high), dir = 0 (CW) ou 1 (CCW), speed = 0–3000 (RPM)
void set_home_parameters(int8_t moteur_index, uint8_t trig, uint8_t dir, uint16_t speed) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = {
        trig,
        dir,
        static_cast<uint8_t>(speed & 0xFF),
        static_cast<uint8_t>((speed >> 8) & 0xFF)
    };

    envoyer_commande(moteur, Commande::SetHomeParams, donnees);
}

// ! Définit la position actuelle comme zéro sans mouvement
void set_home_position_zero(int8_t moteur_index) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = {};  // aucun paramètre

    envoyer_commande(moteur, Commande::SetHomeZero, donnees);
}

// ! Restaure les paramètres usine du moteur (attention, irréversible !)
void restore_factory_settings(int8_t moteur_index) {
    Moteur moteur;

    if (moteur_index < 0) {
        moteur.id = 0;  // broadcast
    } else {
        moteur = mot[moteur_index];
    }

    std::vector<uint8_t> donnees = {};  // pas de paramètre

    envoyer_commande(moteur, Commande::RestoreDefaults, donnees);
}