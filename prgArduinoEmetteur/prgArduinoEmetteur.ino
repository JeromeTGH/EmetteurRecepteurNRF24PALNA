/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduinoEmetteur.ino  
  Description :   Programme permettant de réaliser un émetteur radio, à base de module nRF24 + PA + LNA

  Licence :       BY-NC-ND 4.0 CC (https://creativecommons.org/licenses/by-nc-nd/4.0/deed.fr)
  
  Remarques :     - le microcontrôleur utilisé ici sera un ATmega328P (version DIP)
                  - la programmation du µC se fera via l'IDE Arduino, en utilisant FTDI comme passerelle
                  - un sélecteur rotatif à 10 positions permettra de choisir l'une des dix fréquences de transmission possibles
                  - l'émetteur dispose de 4 boutons poussoirs, qui piloteront les 4 relais disposés au niveau du récepteur correspondant

  Dépôt GitHub :  https://github.com/JeromeTGH/EmetteurRecepteurNRF24PALNA (fichiers sources du projet, émetteur + récepteur)

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       07.04.2024
  
*/

// Définition des broches de raccordement à l'ATmega328P
//      Remarque 1 : hors lignes UART (TX/RX) et SPI (MISO/MOSI/SCK)
//      Remarque 2 : les broches A0/A1/A2/A3/A4 de ce programme arduino correspondent respectivement aux broches physiques 23/24/25/26/27 de la puce ATmega328P
//      Remarque 3 : les broches D2/D3/D4/D5/D6/D7/D9/D10 de ce programme arduino correspondent respectivement aux broches physiques 4/5/6/11/12/13/15/16 de l'ATmega328P
#define entreeA0_ATmega328P_lecture_tension_batterie                    A0          // Pour lire la tension (abaissée) de la batterie alimentant le projet
#define entreeA1_ATmega328P_lecture_etat_bouton_poussoir_voie_1         A1          // Pour lire l'état du bouton-poussoir de la voie 1 (0=appuyé / 1=relâché)
#define entreeA2_ATmega328P_lecture_etat_bouton_poussoir_voie_2         A2          // Pour lire l'état du bouton-poussoir de la voie 2 (0=appuyé / 1=relâché)
#define entreeA3_ATmega328P_lecture_etat_bouton_poussoir_voie_3         A3          // Pour lire l'état du bouton-poussoir de la voie 3 (0=appuyé / 1=relâché)
#define entreeA4_ATmega328P_lecture_etat_bouton_poussoir_voie_4         A4          // Pour lire l'état du bouton-poussoir de la voie 4 (0=appuyé / 1=relâché)

#define entreeD2_ATmega328P_lecture_etat_ligne_1_encodeur_10_positions  2           // Pour lire l'état de la ligne de poids 1 de l'encodeur à 10 positions
#define entreeD3_ATmega328P_lecture_etat_ligne_2_encodeur_10_positions  3           // Pour lire l'état de la ligne de poids 2 de l'encodeur à 10 positions
#define entreeD4_ATmega328P_lecture_etat_ligne_4_encodeur_10_positions  4           // Pour lire l'état de la ligne de poids 4 de l'encodeur à 10 positions
#define entreeD5_ATmega328P_lecture_etat_ligne_8_encodeur_10_positions  5           // Pour lire l'état de la ligne de poids 8 de l'encodeur à 10 positions

#define sortieD6_ATmega328P_pilotage_led_indication_batterie_faible     6           // Pour piloter la LED "batterie faible" (0=led éteinte / 1=led allumée)
#define sortieD7_ATmega328P_pilotage_led_indication_programme_demarre   7           // Pour piloter la LED "programme démarré" (0=led éteinte / 1=led allumée)

#define sortieD9_ATmega328P_vers_entree_CE_du_module_NRF24L01_PA_LNA    9           // Pour piloter la ligne "CE" du module NRF24L01+PA+LNA
#define sortieD10_ATmega328P_vers_entree_CSN_du_module_NRF24L01_PA_LNA  10          // Pour piloter la ligne "CSN" du module NRF24L01+PA+LNA

// Définition des autres constantes
#define valeur_en_ohms_resistance_basse_pont_diviseur_de_tension_accu   4700        // Pont diviseur de tension, permettant d'abaisser la tension de l'accu (batterie),
#define valeur_en_ohms_resistance_haute_pont_diviseur_de_tension_accu   10000       // pour que celle-ci ne soit pas trop haute, pour être lue via l'entrée analogique de l'ATmega328P



// ========================
// Initialisation programme
// ========================
void setup() {

    // Configuration des entrées de l'ATmega328P, gérées "manuellement"
    pinMode(entreeA0_ATmega328P_lecture_tension_batterie, INPUT);
    pinMode(entreeA1_ATmega328P_lecture_etat_bouton_poussoir_voie_1, INPUT_PULLUP);             // Activation des pull-up au niveau des boutons poussoirs
    pinMode(entreeA2_ATmega328P_lecture_etat_bouton_poussoir_voie_2, INPUT_PULLUP);             //    (ici : 1=bouton relâché / 0=bouton appuyé)
    pinMode(entreeA3_ATmega328P_lecture_etat_bouton_poussoir_voie_3, INPUT_PULLUP);
    pinMode(entreeA4_ATmega328P_lecture_etat_bouton_poussoir_voie_4, INPUT_PULLUP);
    pinMode(entreeD2_ATmega328P_lecture_etat_ligne_1_encodeur_10_positions, INPUT_PULLUP);      // Activation des pull-up au niveau des lignes de l'encodeur à 10 positions
    pinMode(entreeD3_ATmega328P_lecture_etat_ligne_2_encodeur_10_positions, INPUT_PULLUP);      //    (ici : 1=ligne inactive / 0=ligne active)
    pinMode(entreeD4_ATmega328P_lecture_etat_ligne_4_encodeur_10_positions, INPUT_PULLUP);
    pinMode(entreeD5_ATmega328P_lecture_etat_ligne_8_encodeur_10_positions, INPUT_PULLUP);

    // Configuration des sorties de l'ATmega328P, gérées "manuellement"
    pinMode(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, OUTPUT);
    pinMode(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, OUTPUT);

    // Définition des états initiaux des lignes de sorties
    digitalWrite(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, LOW);             // Led "batterie faible" éteinte, pour l'instant
    digitalWrite(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, LOW);           // Led "programme démarré" éteinte, pour l'instant

    // Petite pause, avant de passer à la boucle LOOP
    delay(100);

}


// =================
// Boucle principale
// =================
void loop() {
  
}
