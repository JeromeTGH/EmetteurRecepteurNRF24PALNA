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
  Description :   Programme permettant de réaliser un récepteur radio, à base de module nRF24+PA+LNA

  Licence :       BY-NC-ND 4.0 CC (https://creativecommons.org/licenses/by-nc-nd/4.0/deed.fr)

  Remarques :     - le microcontrôleur utilisé ici sera un ATmega328P (version DIP)
                  - la programmation du µC se fera via l'IDE Arduino, en utilisant un FTDI comme passerelle
                  - un sélecteur rotatif à 10 positions permettra de choisir l'une des dix canaux de transmission possibles
                  - le récepteur dispose de 4 relais, respectivement pilotés par les 4 boutons poussoirs de l'émetteur

  Dépôt GitHub :  https://github.com/JeromeTGH/EmetteurRecepteurNRF24PALNA (fichiers sources du projet, émetteur + récepteur)

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       07.04.2024

*/

// Inclusion de la librairie nRF24 (auteur : https://github.com/nRF24/RF24)
#include <RF24.h>

// ************************************************************************************************************************************
// Partie débogage, au besoin
#define DEBUG 1                                       // Mettre à 0 ou à 1 pour afficher ou non les messages de debug sur le port série

#if DEBUG
  #define DEBUGMSG(message) Serial.print(message)     // Pour info, ajouter un \r\n à la fin du message, pour un retour à la ligne
#else
  #define DEBUGMSG(message)
#endif
// ************************************************************************************************************************************


// Définition des broches de raccordement à l'ATmega328P
//      Remarque 1 : hors lignes UART (TX/RX) et SPI (MISO/MOSI/SCK)
//      Remarque 2 : pour info, les broches A0/A1/A2/A3/A4/A5 de ce programme arduino correspondent respectivement aux broches physiques 23/24/25/26/27/28 de la puce ATmega328P
//      Remarque 3 : pour info, les broches D2/D3/D4/D5/D6/D7/D9/D10 de ce programme arduino correspondent respectivement aux broches physiques 4/5/6/11/12/13/15/16 de l'ATmega328P
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


// Définition du canal de communication "de base" (définissant la fréquence de base, à laquelle l'émetteur et le récepteur vont communiquer)
#define canal_de_communication_de_base_pour_transmissions_NRF24         79          // Nota 1 : 126 canaux sont disposibles (de 0 à 125, permettant d'échanger de 2,4GHz à 2,525GHz inclus)
// Nota 1 : les modules nRF24 peuvent recevoir sur l'un des 126 canaux à disposition, allant du canal 0 au canal 125
// Nota 2 : la valeur à mettre ici doit être inférieure ou égale à 116 ici, du fait qu'on peut rajouter jusqu'à 9 "crans", sur le sélecteur à 10 positions soudé sur PCB
// Nota 3 : ici j'ai mis 79 par défaut, ce qui est une valeur totalement arbitraire (à ajuster comme bon nous semble, du moment qu'on est entre 0 et 116 inclus)

// Définition du nom du tunnel de communication
const byte nom_de_notre_tunnel_de_communication[6] = "ERJT1";     // Attention : 5 caractères max ici (devra être identique, du côté émetteur)

// Définition des messages attendus, selon quel bouton poussoir est actionné au niveau de l'émetteur (de 1 à 32 caractères, maximum)
const char message_si_bouton_poussoir_1_appuye[] = "Bouton_1_appuye";
const char message_si_bouton_poussoir_2_appuye[] = "Bouton_2_appuye";
const char message_si_bouton_poussoir_3_appuye[] = "Bouton_3_appuye";
const char message_si_bouton_poussoir_4_appuye[] = "Bouton_4_appuye";

// Instanciation de la librairie RF24
RF24 module_nrf24(sortieD9_ATmega328P_vers_entree_CE_du_module_NRF24L01_PA_LNA, sortieD10_ATmega328P_vers_entree_CSN_du_module_NRF24L01_PA_LNA);

// Variables globales
char message_recu[32];
bool relais_1_actif = false;
bool relais_2_actif = false;
bool relais_3_actif = false;
bool relais_4_actif = false;
uint8_t precedente_valeur_du_canal_choisi_sur_PCB;


// ========================
// Initialisation programme
// ========================
void setup() {

  #if DEBUG
    Serial.begin(9600);
  #endif
  DEBUGMSG(F("Démarrage du programme ...\r\n"));

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
  digitalWrite(sortieD8_ATmega328P_pilotage_led_indication_programme_demarre, LOW);           // Led "programme démarré" éteinte, pour commencer

  // Test de la LED "programme démarré" (clignotements rapides), pour que l'utilisateur puisse vérifier qu'elle fonctionne bien
  faireClignoterLedAuDemarrage();

  // Temporaire : utilitaire de test des relais (avec réinitialisation préalable de leur état)
  utilitaireDeTestRelais();

  // Détermine le canal de communication à utiliser (avec initialisation de la "valeur précédente", pour démarrer)
  precedente_valeur_du_canal_choisi_sur_PCB = retourneValeurDuCanalChoisi();
  DEBUGMSG(F("Canal sélectionné sur PCB = "));
  DEBUGMSG(precedente_valeur_du_canal_choisi_sur_PCB);
  DEBUGMSG(F(" (de 0 à 9)\r\n"));
  uint8_t valeur_du_canal_de_communication_reel = canal_de_communication_de_base_pour_transmissions_NRF24 + precedente_valeur_du_canal_choisi_sur_PCB;
  DEBUGMSG(F("Canal de transmission \"réel\" = "));
  DEBUGMSG(valeur_du_canal_de_communication_reel);
  DEBUGMSG(F(" (fréq= "));
  DEBUGMSG(2400 + valeur_du_canal_de_communication_reel);     // Le canal 0 correspondant à une fréquence de 2,4 GHz (soit 2400 MHz)
  DEBUGMSG(F(" MHz)\r\n"));
  
  // Initialisation du module nRF24L01
  if (!module_nrf24.begin()) {
    // En cas d'échec d'initialisation : arrêt du programme
    DEBUGMSG(F("\r\n"));
    DEBUGMSG(F("Initialisation du module nRF24 impossible. Arrêt du programme.)\r\n"));
    delay(300);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
  }

  // Paramétrage de la librairie RF24
  module_nrf24.setAddressWidth(5);                                                    // Fixation de la longueur d'adresse du tunnel (5 octets, par défaut)
  module_nrf24.setChannel(valeur_du_canal_de_communication_reel);                     // Fixation du canal de transmission, pour le récepteur
  module_nrf24.setDataRate(RF24_250KBPS);                                             // Vitesse de communication RF24_250KBPS, RF24_1MBPS, ou RF24_2MBPS (en sachant que "transmettre moins vite permet d'aller plus loin")
  module_nrf24.openReadingPipe(0, nom_de_notre_tunnel_de_communication);              // Ouverture du tunnel de transmission en LECTURE, avec le "nom" qu'on lui a donné (via le "pipe 0", par exemple)
  module_nrf24.setPALevel(RF24_PA_MAX);                                               // Niveau RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, ou RF24_PA_MAX (mis au max, pour pouvoir communiquer le plus loin possible)
  module_nrf24.startListening();                                                      // Activation de l'écoute, car ici c'est le récepteur !

  // Petite pause de stabilisation
  delay(300);

  // Allumage de la LED "programme démarré", et passage à la boucle LOOP
  digitalWrite(sortieD8_ATmega328P_pilotage_led_indication_programme_demarre, HIGH);
  DEBUGMSG(F("Programme démarré.\r\n"));
  DEBUGMSG(F("\r\n"));

}


// =================
// Boucle principale
// =================
void loop() {

  // Déclaration d'une variable, qui contiendra le numéro de tunnel sur lequel sont reçues des infos
  uint8_t numero_de_tunnel;

  // On regarde si des messages sont en attente de lecture
  if (module_nrf24.available(&numero_de_tunnel)) {

    // Ici, on va travailler avec le tunnel 0 (décidé juste au dessus, avec la fonction "openReadingPipe(0, ...)", donc on ne prendre en compte que les données reçues via ce pipe)
    if (numero_de_tunnel == 0) {

      // On stocke X caractères, dans la variables nommée "message_recu" (X étant le nombre de caractères max à recevoir ; 32 au maximum)
      module_nrf24.read(&message_recu, sizeof(message_recu));

      // --------------------------------------------------------
      // On regarde si le message est en rapport avec le relais 1
      // --------------------------------------------------------
      if (String(message_recu) == String(message_si_bouton_poussoir_1_appuye)) {
        if (relais_1_actif) {
          // Si le relais est actif, alors on le relâche (impulsion sur sa "ligne de désactivation")
          digitalWrite(sortieA1_ATmega328P_desactivation_relais_1, HIGH);     delay(100);
          digitalWrite(sortieA1_ATmega328P_desactivation_relais_1, LOW);      delay(10);
          relais_1_actif = false;
          DEBUGMSG(F("Relais 1 désactivé\r\n"));
        } else {
          // Si le relais est relâché, alors on l'active (impulsion sur sa "ligne d'activation")
          digitalWrite(sortieA0_ATmega328P_activation_relais_1, HIGH);        delay(100);
          digitalWrite(sortieA0_ATmega328P_activation_relais_1, LOW);         delay(10);
          relais_1_actif = true;
          DEBUGMSG(F("Relais 1 activé\r\n"));
        }
      }

      // --------------------------------------------------------
      // On regarde si le message est en rapport avec le relais 2
      // --------------------------------------------------------
      if (String(message_recu) == String(message_si_bouton_poussoir_2_appuye)) {
        if (relais_2_actif) {
          // Si le relais est actif, alors on le relâche (impulsion sur sa "ligne de désactivation")
          digitalWrite(sortieA3_ATmega328P_desactivation_relais_2, HIGH);     delay(100);
          digitalWrite(sortieA3_ATmega328P_desactivation_relais_2, LOW);      delay(10);
          relais_2_actif = false;
          DEBUGMSG(F("Relais 2 désactivé\r\n"));
        } else {
          // Si le relais est relâché, alors on l'active (impulsion sur sa "ligne d'activation")
          digitalWrite(sortieA2_ATmega328P_activation_relais_2, HIGH);        delay(100);
          digitalWrite(sortieA2_ATmega328P_activation_relais_2, LOW);         delay(10);
          relais_2_actif = true;
          DEBUGMSG(F("Relais 2 activé\r\n"));
        }
      }

      // --------------------------------------------------------
      // On regarde si le message est en rapport avec le relais 3
      // --------------------------------------------------------
      if (String(message_recu) == String(message_si_bouton_poussoir_3_appuye)) {
        if (relais_3_actif) {
          // Si le relais est actif, alors on le relâche (impulsion sur sa "ligne de désactivation")
          digitalWrite(sortieA5_ATmega328P_desactivation_relais_3, HIGH);     delay(100);
          digitalWrite(sortieA5_ATmega328P_desactivation_relais_3, LOW);      delay(10);
          relais_3_actif = false;
          DEBUGMSG(F("Relais 3 désactivé\r\n"));
        } else {
          // Si le relais est relâché, alors on l'active (impulsion sur sa "ligne d'activation")
          digitalWrite(sortieA4_ATmega328P_activation_relais_3, HIGH);        delay(100);
          digitalWrite(sortieA4_ATmega328P_activation_relais_3, LOW);         delay(10);
          relais_3_actif = true;
          DEBUGMSG(F("Relais 3 activé\r\n"));
        }
      }

      // --------------------------------------------------------
      // On regarde si le message est en rapport avec le relais 4
      // --------------------------------------------------------
      if (String(message_recu) == String(message_si_bouton_poussoir_4_appuye)) {
        if (relais_4_actif) {
          // Si le relais est actif, alors on le relâche (impulsion sur sa "ligne de désactivation")
          digitalWrite(sortieD7_ATmega328P_desactivation_relais_4, HIGH);     delay(100);
          digitalWrite(sortieD7_ATmega328P_desactivation_relais_4, LOW);      delay(10);
          relais_4_actif = false;
          DEBUGMSG(F("Relais 4 désactivé\r\n"));
        } else {
          // Si le relais est relâché, alors on l'active (impulsion sur sa "ligne d'activation")
          digitalWrite(sortieD6_ATmega328P_activation_relais_4, HIGH);        delay(100);
          digitalWrite(sortieD6_ATmega328P_activation_relais_4, LOW);         delay(10);
          relais_4_actif = true;
          DEBUGMSG(F("Relais 4 activé\r\n"));
        }
      }
    }
  }

  // Vérification si changement manuel de canal, sur le PCB
  uint8_t valeur_de_canal_relue = retourneValeurDuCanalChoisi();
  if(valeur_de_canal_relue != precedente_valeur_du_canal_choisi_sur_PCB) {
    delay(150);                                                     // Anti-rebond, pour filtrer les éventuels états indésirables,
    valeur_de_canal_relue = retourneValeurDuCanalChoisi();          // puis relecture de la valeur après coup
    uint8_t valeur_du_nouveau_canal = canal_de_communication_de_base_pour_transmissions_NRF24 + valeur_de_canal_relue;
    module_nrf24.setChannel(valeur_du_nouveau_canal);
    precedente_valeur_du_canal_choisi_sur_PCB = valeur_de_canal_relue;
    DEBUGMSG(F("\r\n"));
    DEBUGMSG(F("Nouveau canal sélectionné sur PCB = "));
    DEBUGMSG(valeur_de_canal_relue);
    DEBUGMSG(F(" (de 0 à 9)\r\n"));
    DEBUGMSG(F("Nouveau canal de transmission \"réel\" = "));
    DEBUGMSG(valeur_du_nouveau_canal);
    DEBUGMSG(F(" (fréq= "));
    DEBUGMSG(2400 + valeur_du_nouveau_canal);       // Le canal 0 correspondant à 2400 MHz, pour rappel
    DEBUGMSG(F(" MHz)\r\n"));
    DEBUGMSG(F("\r\n"));
  }
  
}


// =======================================
// Fonction : faireClignoterLedAuDemarrage
// =======================================
//      Permet de faire clignoter la led "programme démarré" X fois, au démarrage
void faireClignoterLedAuDemarrage() {

  DEBUGMSG(F("Test de la LED...\r\n"));

  for (uint8_t i = 0; i < 5 ; i++) {
    // Allumage de la LED
    digitalWrite(sortieD8_ATmega328P_pilotage_led_indication_programme_demarre, HIGH);
    delay(50);

    // Extinction LED
    digitalWrite(sortieD8_ATmega328P_pilotage_led_indication_programme_demarre, LOW);
    delay(100);
  }

}


// =================================
// Fonction : utilitaireDeTestRelais
// =================================
//      Actionne les relais, pour faire des tests (les met sur "OFF", plus les active/désactive l'un après l'autre, avec 1 seconde d'écart)
void utilitaireDeTestRelais() {

  DEBUGMSG(F("Test des relais...\r\n"));

  // Pour commencer, on met tous les relais en position "OFF" (en faisant une "impulsion" HIGH puis LOW, sur chaque ligne de commande)
  digitalWrite(sortieA1_ATmega328P_desactivation_relais_1, HIGH);
  digitalWrite(sortieA3_ATmega328P_desactivation_relais_2, HIGH);
  digitalWrite(sortieA5_ATmega328P_desactivation_relais_3, HIGH);
  digitalWrite(sortieD7_ATmega328P_desactivation_relais_4, HIGH);
  delay(100);
  digitalWrite(sortieA1_ATmega328P_desactivation_relais_1, LOW);
  digitalWrite(sortieA3_ATmega328P_desactivation_relais_2, LOW);
  digitalWrite(sortieA5_ATmega328P_desactivation_relais_3, LOW);
  digitalWrite(sortieD7_ATmega328P_desactivation_relais_4, LOW);
  delay(900);

  // Puis on met tous les relais en position "ON" (impulsion de 100 ms)
  digitalWrite(sortieA0_ATmega328P_activation_relais_1, HIGH);
  digitalWrite(sortieA2_ATmega328P_activation_relais_2, HIGH);
  digitalWrite(sortieA4_ATmega328P_activation_relais_3, HIGH);
  digitalWrite(sortieD6_ATmega328P_activation_relais_4, HIGH);
  delay(100);
  digitalWrite(sortieA0_ATmega328P_activation_relais_1, LOW);
  digitalWrite(sortieA2_ATmega328P_activation_relais_2, LOW);
  digitalWrite(sortieA4_ATmega328P_activation_relais_3, LOW);
  digitalWrite(sortieD6_ATmega328P_activation_relais_4, LOW);
  delay(900);

  // Et enfin, on remet les relais en position "OFF"
  digitalWrite(sortieA1_ATmega328P_desactivation_relais_1, HIGH);
  digitalWrite(sortieA3_ATmega328P_desactivation_relais_2, HIGH);
  digitalWrite(sortieA5_ATmega328P_desactivation_relais_3, HIGH);
  digitalWrite(sortieD7_ATmega328P_desactivation_relais_4, HIGH);
  delay(100);
  digitalWrite(sortieA1_ATmega328P_desactivation_relais_1, LOW);
  digitalWrite(sortieA3_ATmega328P_desactivation_relais_2, LOW);
  digitalWrite(sortieA5_ATmega328P_desactivation_relais_3, LOW);
  digitalWrite(sortieD7_ATmega328P_desactivation_relais_4, LOW);
  delay(900);

}


// ======================================
// Fonction : retourneValeurDuCanalChoisi
// ======================================
//      Nota : renvoi la valeur (pouvant aller de 0 à 9 inclus) de l'encodeur rotatif, soudé sur le PCB
uint8_t retourneValeurDuCanalChoisi() {

  // Variable qui sera retournée
  uint8_t valeur_du_canal_choisi;

  // Lecture des 4 entrées de l'encodeur rotatif
  uint8_t valeur_ligne_1 = digitalRead(entreeD2_ATmega328P_lecture_etat_ligne_1_encodeur_10_positions);
  uint8_t valeur_ligne_2 = digitalRead(entreeD3_ATmega328P_lecture_etat_ligne_2_encodeur_10_positions);
  uint8_t valeur_ligne_4 = digitalRead(entreeD4_ATmega328P_lecture_etat_ligne_4_encodeur_10_positions);
  uint8_t valeur_ligne_8 = digitalRead(entreeD5_ATmega328P_lecture_etat_ligne_8_encodeur_10_positions);

  // Détermination de la valeur décimale correspondante
  valeur_du_canal_choisi = 0;
  if (valeur_ligne_1 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 1;
  if (valeur_ligne_2 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 2;
  if (valeur_ligne_4 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 4;
  if (valeur_ligne_8 == LOW) valeur_du_canal_choisi = valeur_du_canal_choisi + 8;

  // Retourne la valeur décimale correspondant au canal choisi sur l'encodeur
  return valeur_du_canal_choisi;
}
