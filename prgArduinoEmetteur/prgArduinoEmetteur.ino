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
  Description :   Programme permettant de réaliser un émetteur radio, à base de module nRF24+PA+LNA

  Licence :       BY-NC-ND 4.0 CC (https://creativecommons.org/licenses/by-nc-nd/4.0/deed.fr)

  Remarques :     - le microcontrôleur utilisé ici sera un ATmega328P (version DIP)
                  - la programmation du µC se fera via l'IDE Arduino, en utilisant un FTDI comme passerelle
                  - un sélecteur rotatif à 10 positions permettra de choisir l'une des dix fréquences de transmission possibles
                  - l'émetteur dispose de 4 boutons poussoirs, qui piloteront les 4 relais disposés au niveau du récepteur

  Dépôt GitHub :  https://github.com/JeromeTGH/EmetteurRecepteurNRF24PALNA (fichiers sources du projet, émetteur + récepteur)

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       07.04.2024

*/

// Inclusion de la librairie nRF24 (auteur : https://github.com/nRF24/RF24)
#include <RF24.h>

// *************************************************************************************************************************************
// Partie débogage, au besoin
#define DEBUG 1                                       // Mettre à 0 ou à 1 pour afficher ou non les messages en retour sur le port série

#if DEBUG
  #define DEBUGMSG(message) Serial.print(message)     // Pour info, ajouter \r\n à la fin du message, pour un retour à la ligne
#else
  #define DEBUGMSG(message)
#endif
// *************************************************************************************************************************************


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

// Définition des valeurs définissant le pont diviseur de tension, donnant une image abaissée de la tension de la batterie alimentant cet émetteur
#define valeur_en_ohms_resistance_basse_pont_diviseur_de_tension_accu   47000       // Pont diviseur de tension, permettant d'abaisser la tension de l'accu (batterie),
#define valeur_en_ohms_resistance_haute_pont_diviseur_de_tension_accu   100000      // pour que celle-ci ne soit pas trop haute, pour être lue via l'entrée analogique de l'ATmega328P

// Définition du canal de communication "de base" (la fréquence de base, à laquelle l'émetteur et le récepteur vont communiquer)
#define canal_de_communication_de_base_pour_transmissions_NRF24         79          // Nota 1 : 126 canaux sont disposibles (de 0 à 125, permettant d'échanger de 2,4GHz à 2,525GHz inclus)
// Nota 2 : la valeur à mettre ici doit être comprise entre 0 et 116, puisqu'on pourra ajouter entre 0 et 9 "crans" (via le sélecteur à 10 positions)
// Nota 3 : ici j'ai mis 79 par défaut, ce qui est une valeur arbitraire (à ajuster personnellement, en fait)

// Définition du nom du tunnel de communication
const byte nom_de_notre_tunnel_de_communication[6] = "ERJT1";     // Attention : 5 caractères max ici (devra être identique, côté émetteur et côté récepteur)


// Définition des messages à émettre, suivant quel bouton poussoir est actionné (de 1 à 32 caractères, maximum)
const char message_si_bouton_poussoir_1_appuye[] = "Bouton_1_appuye";
const char message_si_bouton_poussoir_2_appuye[] = "Bouton_2_appuye";
const char message_si_bouton_poussoir_3_appuye[] = "Bouton_3_appuye";
const char message_si_bouton_poussoir_4_appuye[] = "Bouton_4_appuye";

// Instanciation de la librairie RF24
RF24 module_nrf24(sortieD9_ATmega328P_vers_entree_CE_du_module_NRF24L01_PA_LNA, sortieD10_ATmega328P_vers_entree_CSN_du_module_NRF24L01_PA_LNA);


// ========================
// Initialisation programme
// ========================
void setup() {

  #if DEBUG
    Serial.begin(9600);
  #endif
  DEBUGMSG(F("Démarrage du programme ...\r\n"));

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

  // Clignotage LEDs, avant tentative de démarrage module nRF24
  faireClignoterLedsAuDemarrage();

  // Détermine le canal de communication à utiliser
  uint8_t valeur_du_canal_choisi_sur_PCB = retourneValeurDuCanalChoisi();
  DEBUGMSG(F("Canal sélectionné sur PCB = "));
  DEBUGMSG(valeur_du_canal_choisi_sur_PCB);
  DEBUGMSG(F(" (de 0 à 9)\r\n"));
  uint8_t valeur_du_canal_de_communication_reel = canal_de_communication_de_base_pour_transmissions_NRF24 + valeur_du_canal_choisi_sur_PCB;

  // Initialisation du module nRF24L01
  if (!module_nrf24.begin()) {
    // En cas d'échec d'initialisation : boucle infinie / suspension du programme
    while (1) {}
  }

  // Paramétrage de la librairie RF24
  module_nrf24.setAddressWidth(5);                                                    // Fixation de la longueur d'adresse du tunnel (5 octets, par défaut)
  module_nrf24.setChannel(valeur_du_canal_de_communication_reel);                     // Fixation du canal de transmission, pour l'émetteur
  module_nrf24.setDataRate(RF24_250KBPS);                                             // Vitesse de communication RF24_250KBPS, RF24_1MBPS, ou RF24_2MBPS ("transmettre moins vite permet d'aller plus loin")
  module_nrf24.openWritingPipe(nom_de_notre_tunnel_de_communication);                 // Ouverture du tunnel de transmission en ÉCRITURE, avec le "nom" qu'on lui a donné (via le "pipe 0", obligatoirement en émission)
  module_nrf24.setPALevel(RF24_PA_MAX);                                               // Niveau RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, ou RF24_PA_MAX (mise au max, pour pouvoir communiquer le plus loin possible)
  module_nrf24.stopListening();                                                       // Arrêt de l'écoute, car ici c'est l'émetteur, donc on va émettre !

  // Petite pause de stabilisation
  delay(300);

  // Allumage de la LED "programme démarré", et passage à la boucle LOOP
  digitalWrite(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, HIGH);
  DEBUGMSG(F("Programme démarré.\r\n"));
  DEBUGMSG(F("\r\n"));

}


// =================
// Boucle principale
// =================
void loop() {

  // Traitement du bouton 1
  if (estEnfonceCeBoutonPoussoir(1)) {
    while (estEnfonceCeBoutonPoussoir(1)) {
      delay(10); // Attente que le bouton soit relâché (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_1_appuye, sizeof(message_si_bouton_poussoir_1_appuye));      // Envoi du message correspondant
    delay(20);                                                                                                  // Petit "anti-rebond logiciel" (20 ms de durée)
    envoieMessageSurPortSerie(message_si_bouton_poussoir_1_appuye);
  }

  // Traitement du bouton 2
  if (estEnfonceCeBoutonPoussoir(2)) {
    while (estEnfonceCeBoutonPoussoir(2)) {
      delay(10); // Attente que le bouton soit relâché (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_2_appuye, sizeof(message_si_bouton_poussoir_2_appuye));      // Envoi du message correspondant
    delay(20);                                                                                                  // Petit "anti-rebond logiciel" (20 ms de durée)
    envoieMessageSurPortSerie(message_si_bouton_poussoir_2_appuye);
  }

  // Traitement du bouton 3
  if (estEnfonceCeBoutonPoussoir(3)) {
    while (estEnfonceCeBoutonPoussoir(3)) {
      delay(10); // Attente que le bouton soit relâché (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_3_appuye, sizeof(message_si_bouton_poussoir_3_appuye));      // Envoi du message correspondant
    delay(20);                                                                                                  // Petit "anti-rebond logiciel" (20 ms de durée)
    envoieMessageSurPortSerie(message_si_bouton_poussoir_3_appuye);
  }

  // Traitement du bouton 4
  if (estEnfonceCeBoutonPoussoir(4)) {
    while (estEnfonceCeBoutonPoussoir(4)) {
      delay(10); // Attente que le bouton soit relâché (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_4_appuye, sizeof(message_si_bouton_poussoir_4_appuye));      // Envoi du message correspondant
    delay(20);                                                                                                  // Petit "anti-rebond logiciel" (20 ms de durée)
    envoieMessageSurPortSerie(message_si_bouton_poussoir_4_appuye);
  }

  // Petite pause, avant de reboucler
  delay(50);

}


// =====================================
// Fonction : estEnfonceCeBoutonPoussoir
// =====================================
//      Ici, on lit l'état du bouton poussoir demandé (1 à 4, inclus)
//      La valeur lue vaut HIGH si le bouton est relâché (dû à la résistance pull-up associée), ou LOW si le bouton est appuyé (car il y a mise à la masse)
//      On inverse donc ce résultat lu, pour retourner HIGH lorsque ce bouton est enfoncé, et LOW lorsqu'il est relâché
bool estEnfonceCeBoutonPoussoir(uint8_t numero_de_bouton_poussoir) {

  if (numero_de_bouton_poussoir == 1)
    return !digitalRead(entreeA1_ATmega328P_lecture_etat_bouton_poussoir_voie_1);

  if (numero_de_bouton_poussoir == 2)
    return !digitalRead(entreeA2_ATmega328P_lecture_etat_bouton_poussoir_voie_2);

  if (numero_de_bouton_poussoir == 3)
    return !digitalRead(entreeA3_ATmega328P_lecture_etat_bouton_poussoir_voie_3);

  if (numero_de_bouton_poussoir == 4)
    return !digitalRead(entreeA4_ATmega328P_lecture_etat_bouton_poussoir_voie_4);

}


// ========================================
// Fonction : faireClignoterLedsAuDemarrage
// ========================================
//      Permet de faire clignoter les leds X fois, au démarrage
void faireClignoterLedsAuDemarrage() {

  DEBUGMSG(F("Test des LEDs...\r\n"));

  for (uint8_t i = 0; i < 5 ; i++) {
    // Allumage des LEDs
    digitalWrite(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, HIGH);
    digitalWrite(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, HIGH);
    delay(50);

    // Extinntion LEDs
    digitalWrite(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, LOW);
    digitalWrite(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, LOW);
    delay(100);
  }

}


// ======================================
// Fonction : retourneValeurDuCanalChoisi
// ======================================
//      Nota : l'encodeur rotatif permettant de sélectionner un canal va de 0 à 9 (ce qui "s'ajoutera" à la fréquence de base choisie) ;
//             cette valeur est lue au format binaire, avec poids associés (1, 2, 4, ou 8, selon la ligne lue)
uint8_t retourneValeurDuCanalChoisi() {

  // Variable qui sera retournée
  uint8_t valeur_du_canal_choisi;

  // Lecture des 4 entrées de l'encodeur rotatif
  uint8_t valeur_ligne_1 = digitalRead(entreeD2_ATmega328P_lecture_etat_ligne_1_encodeur_10_positions);
  uint8_t valeur_ligne_2 = digitalRead(entreeD3_ATmega328P_lecture_etat_ligne_2_encodeur_10_positions);
  uint8_t valeur_ligne_4 = digitalRead(entreeD4_ATmega328P_lecture_etat_ligne_4_encodeur_10_positions);
  uint8_t valeur_ligne_8 = digitalRead(entreeD5_ATmega328P_lecture_etat_ligne_8_encodeur_10_positions);

  // Détermination de la valeur décimale
  valeur_du_canal_choisi = 0;
  if (valeur_ligne_1 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 1;
  if (valeur_ligne_2 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 2;
  if (valeur_ligne_4 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 4;
  if (valeur_ligne_8 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 8;

  // Retourne la valeur décimale correspondant au canal choisi, sur l'encodeur
  return valeur_du_canal_choisi;
}

// ======================================
// Fonction : retourneValeurDuCanalChoisi
// ======================================
//      Nota : l'encodeur rotatif permettant de sélectionner un canal va de 0 à 9 (ce qui "s'ajoutera" à la fréquence de base choisie) ;
//             cette valeur est lue au format binaire, avec poids associés (1, 2, 4, ou 8, selon la ligne lue)
void envoieMessageSurPortSerie(char msg[]) {
  DEBUGMSG(F("Message \""));
  DEBUGMSG(msg);
  DEBUGMSG(F("\" envoyé !\r\n"));
}
