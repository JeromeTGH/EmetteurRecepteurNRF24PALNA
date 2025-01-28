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
                  - un sélecteur rotatif à 10 positions permettra de choisir l'une des dix canaux de transmission possibles
                  - l'émetteur dispose de 4 boutons poussoirs, qui piloteront les 4 relais disposés au niveau du récepteur

  Dépôt GitHub :  https://github.com/JeromeTGH/EmetteurRecepteurNRF24PALNA (fichiers sources du projet, émetteur + récepteur)

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       07.04.2024

*/

// Inclusion des librairies nécessaires
#include <RF24.h>                                     // Librairie nRF24 (auteur : https://github.com/nRF24/RF24)
#include <avr/sleep.h>                                // Pour le "sleep mode", en cas de batterie trop faible

// ************************************************************************************************************************************
// Partie débogage, au besoin
#define DEBUG 1                                       // Mettre à 0 ou à 1 pour afficher ou non les messages de debug sur le port série

#if DEBUG
  #define DEBUGMSG(message) Serial.print(message)     // Pour info, ajouter \r\n à la fin du message, pour un retour à la ligne
#else
  #define DEBUGMSG(message)
#endif
// ************************************************************************************************************************************


// Définition des broches de raccordement à l'ATmega328P
//      Remarque 1 : hors lignes UART (RX/TX) et SPI (MISO/MOSI/SCK)
//      Remarque 2 : pour info, les broches A0/A1/A2/A3/A4 de ce programme arduino correspondent respectivement aux broches physiques 23/24/25/26/27 de la puce ATmega328P
//      Remarque 3 : pour info, les broches D2/D3/D4/D5/D6/D7/D9/D10 de ce programme arduino correspondent respectivement aux broches physiques 4/5/6/11/12/13/15/16 de l'ATmega328P
#define entreeA0_ATmega328P_lecture_tension_batterie                    A0          // Pour lire la tension (abaissée) de la batterie alimentant ce projet
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
#define valeur_en_ohms_resistance_basse_pont_diviseur_de_tension_accu   47000       // Pont diviseur de tension, permettant d'abaisser la tension de l'accu (batterie LiPo 3S),
#define valeur_en_ohms_resistance_haute_pont_diviseur_de_tension_accu   100000      // pour que celle-ci ne soit pas trop haute, pour être lue via l'entrée analogique de l'ATmega328P

// Définition du canal de communication "de base" (définissant la fréquence de base, à laquelle l'émetteur et le récepteur vont communiquer)
#define canal_de_communication_de_base_pour_transmissions_NRF24         79          // Nota 1 : 126 canaux sont disposibles (de 0 à 125, permettant d'échanger de 2,4GHz à 2,525GHz inclus)
// Nota 1 : les modules nRF24 peuvent émettre sur l'un des 126 canaux à disposition, allant du canal 0 au canal 125
// Nota 2 : la valeur à mettre ici doit être inférieure ou égale à 116 ici, du fait qu'on peut rajouter jusqu'à 9 "crans", sur le sélecteur à 10 positions soudé sur PCB
// Nota 3 : ici j'ai mis 79 par défaut, ce qui est une valeur totalement arbitraire (à ajuster comme bon nous semble, du moment qu'on est entre 0 et 116 inclus)

// Définition du nom du tunnel de communication
const byte nom_de_notre_tunnel_de_communication[6] = "ERJT1";     // Attention : 5 caractères max ici (devra être identique, du côté récepteur)

// Définition des messages à émettre, suivant quel bouton poussoir est actionné (de 1 à 32 caractères, maximum)
const char message_si_bouton_poussoir_1_appuye[] = "Bouton_1_appuye";
const char message_si_bouton_poussoir_2_appuye[] = "Bouton_2_appuye";
const char message_si_bouton_poussoir_3_appuye[] = "Bouton_3_appuye";
const char message_si_bouton_poussoir_4_appuye[] = "Bouton_4_appuye";

// Instanciation de la librairie RF24
RF24 module_nrf24(sortieD9_ATmega328P_vers_entree_CE_du_module_NRF24L01_PA_LNA, sortieD10_ATmega328P_vers_entree_CSN_du_module_NRF24L01_PA_LNA);

// Variables globales
float tension_accus_estimee;
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
  pinMode(entreeA0_ATmega328P_lecture_tension_batterie, INPUT);
  pinMode(entreeA1_ATmega328P_lecture_etat_bouton_poussoir_voie_1, INPUT_PULLUP);             // Activation des pull-up au niveau des boutons poussoirs
  pinMode(entreeA2_ATmega328P_lecture_etat_bouton_poussoir_voie_2, INPUT_PULLUP);             //    (dans ce cas : bouton relâché = 1 / bouton appuyé = 0)
  pinMode(entreeA3_ATmega328P_lecture_etat_bouton_poussoir_voie_3, INPUT_PULLUP);
  pinMode(entreeA4_ATmega328P_lecture_etat_bouton_poussoir_voie_4, INPUT_PULLUP);
  pinMode(entreeD2_ATmega328P_lecture_etat_ligne_1_encodeur_10_positions, INPUT_PULLUP);      // Activation des pull-up au niveau des lignes de l'encodeur à 10 positions
  pinMode(entreeD3_ATmega328P_lecture_etat_ligne_2_encodeur_10_positions, INPUT_PULLUP);      //    (dans ce cas : ligne inactive = 1 / ligne active = 0)
  pinMode(entreeD4_ATmega328P_lecture_etat_ligne_4_encodeur_10_positions, INPUT_PULLUP);
  pinMode(entreeD5_ATmega328P_lecture_etat_ligne_8_encodeur_10_positions, INPUT_PULLUP);

  // Configuration des sorties de l'ATmega328P, gérées "manuellement"
  pinMode(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, OUTPUT);
  pinMode(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, OUTPUT);

  // Définition des états initiaux des lignes de sorties
  digitalWrite(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, LOW);             // Led "batterie faible" éteinte, pour commencer
  digitalWrite(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, LOW);           // Led "programme démarré" éteinte, pour commencer

  // Test des LEDs (clignotements rapides), pour que l'utilisateur puisse vérifier qu'elles fonctionnent bien
  faireClignoterLedsAuDemarrage();

  // Estimation de la tension batterie (accus lipo 3S, dans mon cas)
  verifieSiTensionAccusSuffisante(true);            // la valeur "true" permet simplement d'afficher la" tension batterie" au démarrage du programme, sur le moniteur série

  // Détermine le canal de communication à utiliser (avec initialisation de la "valeur précédente", pour démarrer)
  precedente_valeur_du_canal_choisi_sur_PCB = retourneValeurDuCanalChoisi();
  DEBUGMSG(F("Canal sélectionné sur PCB = "));
  DEBUGMSG(precedente_valeur_du_canal_choisi_sur_PCB);
  DEBUGMSG(F(" (de 0 à 9)\r\n"));
  uint8_t valeur_du_canal_de_communication_reel = canal_de_communication_de_base_pour_transmissions_NRF24 + precedente_valeur_du_canal_choisi_sur_PCB;
  DEBUGMSG(F("Canal de transmission \"réel\" = "));
  DEBUGMSG(valeur_du_canal_de_communication_reel);
  DEBUGMSG(F(" (fréq= "));
  DEBUGMSG(2400 + valeur_du_canal_de_communication_reel);     // Le canal 0 correspondant à une fréquence de 2,4 GHz (soit 2400 MHz), pour rappel
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
  module_nrf24.setAddressWidth(5);                                          // Fixation de la longueur d'adresse du tunnel (5 octets, par défaut)
  module_nrf24.setChannel(valeur_du_canal_de_communication_reel);           // Fixation du canal de transmission, pour l'émetteur
  module_nrf24.setDataRate(RF24_250KBPS);                                   // Vitesse de communication RF24_250KBPS, RF24_1MBPS, ou RF24_2MBPS (en sachant que "transmettre moins vite permet d'aller plus loin")
  module_nrf24.openWritingPipe(nom_de_notre_tunnel_de_communication);       // Ouverture du tunnel de transmission en ÉCRITURE, avec le "nom" qu'on lui a donné (se fait via le "pipe 0", obligatoirement en émission)
  module_nrf24.setPALevel(RF24_PA_MAX);                                     // Niveau RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, ou RF24_PA_MAX (mis au max, pour pouvoir communiquer le plus loin possible)
  module_nrf24.stopListening();                                             // Arrêt de l'écoute, car ici c'est l'émetteur, donc on va émettre !

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
      delay(10);    // On attend le relâchement du bouton (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_1_appuye, sizeof(message_si_bouton_poussoir_1_appuye));      // Envoi du message correspondant
    delay(10);                                                                                                  // Petit "anti-rebond logiciel" (10 ms de durée)
    ecrireMessageEnvoyeSurPortSerie(message_si_bouton_poussoir_1_appuye);
  }

  // Traitement du bouton 2
  if (estEnfonceCeBoutonPoussoir(2)) {
    while (estEnfonceCeBoutonPoussoir(2)) {
      delay(10);    // On attend le relâchement du bouton (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_2_appuye, sizeof(message_si_bouton_poussoir_2_appuye));      // Envoi du message correspondant
    delay(10);                                                                                                  // Petit "anti-rebond logiciel" (10 ms de durée)
    ecrireMessageEnvoyeSurPortSerie(message_si_bouton_poussoir_2_appuye);
  }

  // Traitement du bouton 3
  if (estEnfonceCeBoutonPoussoir(3)) {
    while (estEnfonceCeBoutonPoussoir(3)) {
      delay(10);    // On attend le relâchement du bouton (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_3_appuye, sizeof(message_si_bouton_poussoir_3_appuye));      // Envoi du message correspondant
    delay(10);                                                                                                  // Petit "anti-rebond logiciel" (10 ms de durée)
    ecrireMessageEnvoyeSurPortSerie(message_si_bouton_poussoir_3_appuye);
  }

  // Traitement du bouton 4
  if (estEnfonceCeBoutonPoussoir(4)) {
    while (estEnfonceCeBoutonPoussoir(4)) {
      delay(10);    // On attend le relâchement du bouton (avec délai de rafraîchissement de 10 ms)
    }
    module_nrf24.write(&message_si_bouton_poussoir_4_appuye, sizeof(message_si_bouton_poussoir_4_appuye));      // Envoi du message correspondant
    delay(10);                                                                                                  // Petit "anti-rebond logiciel" (10 ms de durée)
    ecrireMessageEnvoyeSurPortSerie(message_si_bouton_poussoir_4_appuye);
  }

  // Vérification de la tension des accus, et arrêt du programme si besoin (en allumant au passage la LED "Batterie faible")
  verifieSiTensionAccusSuffisante(false);

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
  
  // Petite pause, avant de reboucler
  delay(50);

}


// =====================================
// Fonction : estEnfonceCeBoutonPoussoir
// =====================================
//      Ici, on lit l'état du bouton poussoir demandé (n° 1 à 4).
//      La valeur lue vaut HIGH/TRUE si le bouton est relâché (dû à la résistance pull-up associée), ou LOW/FALSE si le bouton est appuyé (car il y a mise à la masse).
//      On inverse donc ce résultat lu, pour retourner TRUE lorsque ce bouton est enfoncé, et FALSE lorsqu'il est relâché !
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

    // Extinction LEDs
    digitalWrite(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, LOW);
    digitalWrite(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, LOW);
    delay(100);
  }

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

// ==========================================
// Fonction : ecrireMessageEnvoyeSurPortSerie
// ==========================================
//      Nota : permet d'envoyer un message formaté, sur le port série
void ecrireMessageEnvoyeSurPortSerie(char msg[]) {
  DEBUGMSG(F("Message \""));
  DEBUGMSG(msg);
  DEBUGMSG(F("\" envoyé !\r\n"));
}

// ==============================
// Fonction : calculeTensionAccus
// ==============================
//      Nota : la tension des accus passe physiquement au travers d'un diviseur (abaisseur) de tension,
//             avant d'entrer sur l'entrée A0 de l'ATmega328P ; il faudra donc faire un calcul inverse,
//             pour retrouver/estimer la tension de l'accu, à partir de la tension lue sur A0
void calculeTensionAccus() {
  int valeur_10_bits_mesuree_sur_entree_A0 = analogRead(entreeA0_ATmega328P_lecture_tension_batterie);    // Valeur pouvant aller de 0 à 1024 (10 bits), donc
  float tension_estimee_sur_A0_en_volts = float(valeur_10_bits_mesuree_sur_entree_A0) / 1024.0 * 5.0;     // En idéalisant, la tension de référence de l'ATMEGA328P vaut 5 volts, tout pile !
  
  // D'après la formule du pont diviseur de tension,
  //   V(sortie) = Rinférieure / (Rinf + Rsup) * V(entrée)
  //   d'où V(A0) = R4 / (R3+R4) * V(accus)
  //   d'où V(accus) = V(A0) * (R3+R4) / R4
  // La valeur de V(accus) sera logé dans la variable globale "tension_accus_estimee"

  float valeurDeR3enKohms = valeur_en_ohms_resistance_haute_pont_diviseur_de_tension_accu / 1000.0;
  float valeurDeR4enKohms = valeur_en_ohms_resistance_basse_pont_diviseur_de_tension_accu / 1000.0;
  
  // Remarque : par habitude, j'ai passé les valeurs de résistances en kilo-ohms pour éviter des nombres trop grands
  //            (résultants de calculs intermédiaires), qui pourraient être "tronqués" (overflow) au niveau de l'ATmega328P
  tension_accus_estimee = tension_estimee_sur_A0_en_volts * (valeurDeR3enKohms + valeurDeR4enKohms) / valeurDeR4enKohms;
}



// ==========================================
// Fonction : verifieSiTensionAccusSuffisante
// ==========================================
//    Nota 1 : si la tension est inférieure à un certain seuil, on éteint la LED "Programme démarré", puis on allume la led "Batterie faible"
//    Nota 2 : le paramètre "bAffichage" sert à afficher ou non la valeur de la tension courante sur le moniteur série, lorsque souhaité
void verifieSiTensionAccusSuffisante(boolean bAffichage) {
  calculeTensionAccus();

  // Affichage de la tension lue, si souhaité
  if(bAffichage) {
    DEBUGMSG(F("Tension estimée des accus = "));
    DEBUGMSG(tension_accus_estimee);
    DEBUGMSG(F(" volts\r\n"));
  }

  // Test si tension batterie inférieure à 9 volts (soit 3V par accu, au niveau de la batterie LiPo 3S)
  if(tension_accus_estimee < 9.0) {
    DEBUGMSG(F("Tension des accus trop faible ("));
    DEBUGMSG(tension_accus_estimee);
    DEBUGMSG(F(" V)\r\n"));

    DEBUGMSG(F("  → Extinction LED \"programme démarré\"\r\n"));
    digitalWrite(sortieD7_ATmega328P_pilotage_led_indication_programme_demarre, LOW);

    DEBUGMSG(F("  → Allumage LED \"batterie faible\"\r\n"));
    digitalWrite(sortieD6_ATmega328P_pilotage_led_indication_batterie_faible, HIGH);

    DEBUGMSG(F("  → Arrêt du programme\r\n"));
    delay(300);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
  }
  
}
