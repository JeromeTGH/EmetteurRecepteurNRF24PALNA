/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduinoRecepteur.ino  
  Description :   Programme permettant de réaliser un récepteur radio, à base de module nRF24 + PA + LNA

  Licence :       BY-NC-ND 4.0 CC (https://creativecommons.org/licenses/by-nc-nd/4.0/deed.fr)
  
  Remarques :     - le microcontrôleur utilisé ici sera un ATmega328P (version DIP)
                  - la programmation du µC se fera via l'IDE Arduino, en utilisant FTDI comme passerelle
                  - un sélecteur rotatif à 10 positions permettra de choisir l'une des dix fréquences de transmission possibles
                  - le récepteur dispose de 4 relais, pilotés par les 4 boutons poussoirs de l'émetteur correspondant

  Dépôt GitHub :  https://github.com/JeromeTGH/EmetteurRecepteurNRF24PALNA (fichiers sources du projet, émetteur + récepteur)

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       07.04.2024
  
*/


// Définition des broches de raccordement à l'ATmega328P
//      Remarque 1 : hors lignes UART (TX/RX) et SPI (MISO/MOSI/SCK)
//      Remarque 2 : les broches A0/A1/A2/A3/A4/A5 de ce programme arduino correspondent respectivement aux broches physiques 23/24/25/26/27/28 de la puce ATmega328P
//      Remarque 3 : les broches D2/D3/D4/D5/D6/D7/D9/D10 de ce programme arduino correspondent respectivement aux broches physiques 4/5/6/11/12/13/15/16 de l'ATmega328P
#define sortieA0_ATmega328P_activation_relais_1                         A0          // Pour activer le relais 1
#define sortieA1_ATmega328P_desactivation_relais_1                      A1          // Pour désactiver le relais 1
#define sortieA2_ATmega328P_activation_relais_2                         A2          // Pour activer le relais 2
#define sortieA3_ATmega328P_desactivation_relais_2                      A3          // Pour désactiver le relais 2
#define sortieA4_ATmega328P_activation_relais_3                         A4          // Pour activer le relais 3
#define sortieA5_ATmega328P_desactivation_relais_3                      A5          // Pour désactiver le relais 3
#define sortieD6_ATmega328P_activation_relais_4                         6           // Pour activer le relais 4
#define sortieD7_ATmega328P_desactivation_relais_4                      7           // Pour désactiver le relais 4

#define entreeD2_ATmega328P_lecture_etat_ligne_1_encodeur_10_positions  2           // Pour lire l'état de la ligne de poids 1 de l'encodeur à 10 positions
#define entreeD3_ATmega328P_lecture_etat_ligne_2_encodeur_10_positions  3           // Pour lire l'état de la ligne de poids 2 de l'encodeur à 10 positions
#define entreeD4_ATmega328P_lecture_etat_ligne_4_encodeur_10_positions  4           // Pour lire l'état de la ligne de poids 4 de l'encodeur à 10 positions
#define entreeD5_ATmega328P_lecture_etat_ligne_8_encodeur_10_positions  5           // Pour lire l'état de la ligne de poids 8 de l'encodeur à 10 positions

#define sortieD8_ATmega328P_pilotage_led_indication_programme_demarre   8           // Pour piloter la LED "programme démarré" (0=led éteinte / 1=led allumée)

#define sortieD9_ATmega328P_vers_entree_CE_du_module_NRF24L01_PA_LNA    9           // Pour piloter la ligne "CE" du module NRF24L01+PA+LNA
#define sortieD10_ATmega328P_vers_entree_CSN_du_module_NRF24L01_PA_LNA  10          // Pour piloter la ligne "CSN" du module NRF24L01+PA+LNA


// Définition du canal de communication "de base" (la fréquence de base, à laquelle l'émetteur et le récepteur vont communiquer)
#define canal_de_communication_de_base_pour_transmissions_NRF24         79          // Nota 1 : 126 canaux sont disposibles (de 0 à 125, permettant d'échanger de 2,4GHz à 2,525GHz inclus)
                                                                                    // Nota 2 : la valeur à mettre ici doit être comprise entre 0 et 116, puisqu'on pourra ajouter entre 0 et 9 "crans" (via le sélecteur à 10 positions)
                                                                                    // Nota 3 : ici j'ai mis 79 par défaut, ce qui est une valeur arbitraire (à ajuster personnellement, en fait)



// ========================
// Initialisation programme
// ========================
void setup() {

    // Configuration des entrées de l'ATmega328P, gérées "manuellement"
    pinMode(entreeD2_ATmega328P_lecture_etat_ligne_1_encodeur_10_positions, INPUT_PULLUP);      // Activation des pull-up au niveau des lignes de l'encodeur à 10 positions
    pinMode(entreeD3_ATmega328P_lecture_etat_ligne_2_encodeur_10_positions, INPUT_PULLUP);      //    (ici : 1=ligne inactive / 0=ligne active)
    pinMode(entreeD4_ATmega328P_lecture_etat_ligne_4_encodeur_10_positions, INPUT_PULLUP);
    pinMode(entreeD5_ATmega328P_lecture_etat_ligne_8_encodeur_10_positions, INPUT_PULLUP);

    // Configuration des sorties de l'ATmega328P, gérées "manuellement"
    pinMode(sortieA0_ATmega328P_activation_relais_1, OUTPUT);
    pinMode(sortieA1_ATmega328P_desactivation_relais_1, OUTPUT);
    pinMode(sortieA2_ATmega328P_activation_relais_2, OUTPUT);
    pinMode(sortieA3_ATmega328P_desactivation_relais_2, OUTPUT);
    pinMode(sortieA4_ATmega328P_activation_relais_3, OUTPUT);
    pinMode(sortieA5_ATmega328P_desactivation_relais_3, OUTPUT);
    pinMode(sortieD6_ATmega328P_activation_relais_4, OUTPUT);
    pinMode(sortieD7_ATmega328P_desactivation_relais_4, OUTPUT);
    pinMode(sortieD8_ATmega328P_pilotage_led_indication_programme_demarre, OUTPUT);

    // Définition des états initiaux des lignes de sorties
    digitalWrite(sortieA0_ATmega328P_activation_relais_1, LOW);                                 // Désalimentation de toutes les bobines de relais
    digitalWrite(sortieA1_ATmega328P_desactivation_relais_1, LOW);                              //      → cela se fait via le driver ULN2803
    digitalWrite(sortieA2_ATmega328P_activation_relais_2, LOW);                                 //      → ici, 0=bobine désalimentée / 1=bobine alimentée
    digitalWrite(sortieA3_ATmega328P_desactivation_relais_2, LOW);
    digitalWrite(sortieA4_ATmega328P_activation_relais_3, LOW);
    digitalWrite(sortieA5_ATmega328P_desactivation_relais_3, LOW);
    digitalWrite(sortieD6_ATmega328P_activation_relais_4, LOW);
    digitalWrite(sortieD7_ATmega328P_desactivation_relais_4, LOW);
    digitalWrite(sortieD8_ATmega328P_pilotage_led_indication_programme_demarre, LOW);           // Led "programme démarré" éteinte, pour l'instant

    // Petite pause, avant de passer à la boucle LOOP
    delay(100);

}


// =================
// Boucle principale
// =================
void loop() {
  
}
