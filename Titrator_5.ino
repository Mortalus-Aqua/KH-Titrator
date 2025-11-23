/*

init coté sans anti horaire quand on regarde face aux sorties

wast echan HCL
      PS

*/
#include <SPI.h>
#include <mcp2515.h>
struct can_frame canMsg;
struct can_frame canMsg1;
MCP2515 mcp2515(53);
#include <EEPROM.h>

//RTC
#include <DS3231.h>
#include <Wire.h>

byte ID_Server = 0x0CC;
byte ID = 0x0CD;
unsigned long Time_Send = 0;
unsigned long Time = 0;
unsigned long TimeOut = 5000; //Timeout d'ouverture ou fermeture

//commande
byte SET = 0x01;
byte TITRATION = 0x02; //lancer la titration
byte PRIME = 0x03; //lancer un prime
byte ASK_CONFIG = 0x06;
byte SET_HOUR = 0xA1; //reception
byte SET_DAY = 0xA2;
byte HOUR = 0xA0; //envoi
byte PH = 0xA5; //envoi
byte KH = 0xA6; //envoi
byte CONFIG = 0xBB; //envoi

//ETAT
byte ETAT = 0x00; 
byte ETAT_OK = 0x01; //tout est ok
byte DEPART = 0x02;
byte ENCOURS = 0x03; //En cours de titration ou de prime
byte STOP_ATTENTE = 0x04;
byte ETAT_ERREUR = 0x05;
byte STOP = 0x06;
byte START = 0x07;


int H1[6] = {0,0,0,0,0,0}; //heure à injecter
int M1[6] = {0,0,0,0,0,0}; //minute à injecter
int adrH1[6] = {13,15,17,19,21,23}; //heure à injecter
int adrM1[6] = {25,27,29,31,33,35}; //minute à injecter

byte SET_HORAIRE_t1 = 0x11; //tranche horaire 1
byte SET_HORAIRE_t2 = 0x12; //tranche horaire 2
byte SET_HORAIRE_t3 = 0x13; //tranche horaire 3
byte SET_HORAIRE_t4 = 0x14; //tranche horaire 4
byte SET_HORAIRE_t5 = 0x15; //tranche horaire 5
byte SET_HORAIRE_t6 = 0x16; //tranche horaire 6

//RTC
DS3231 myRTC;
bool century = false;
bool h12Flag;
bool pmFlag;
byte year;
byte month;
byte date;
byte dOW;
byte hour;
byte minute;
byte second;




float pH;
String sensorstring = ""; 

boolean COMMENTAIRE = true;
boolean Flag_STOP = false;
boolean retour = false;


//POMPE AQUARIUM (Extrudeur E0)
const int Pin_STEP_POMPE_AQUA = 26;
const int Pin_DIR_POMPE_AQUA = 28;
const int Pin_EN_POMPE_AQUA = 24;  
int volume_Jour_POMPE_AQUA = 0 ;// volume à ajouter par jour 
int Volume_ajoute_POMPE_AQUA = 0;
int Volume_force_POMPE_AQUA = 0;
float Consigne_nb_pas_POMPE_AQUA= 0;
float nb_pas_POMPE_AQUA = 0;
float Conv_mL_Pas_POMPE_AQUA = 41167; //42650 25.9       2*675816/29.93; //2*422017/20  426609 demi pas/20mL en 60s. 20mL de liquide mesuré pour 60s de débit
unsigned long TimeStep_POMPE_AQUA = 0; //pour se souvenir du dernier micro pas, ne pas toucher
unsigned long periode_Step_POMPE_AQUA = 50; //temps entre deux micro pas, defini le debit
//byte ETAT_pompe_POMPE_AQUA = ENCOURS; 
boolean sortie_POMPE_AQUA = HIGH; //gère l'état du step, ne pas toucher


//POMPE AQUARIUM (Extrudeur E1)
const int Pin_STEP_POMPE_WASTE = 36;
const int Pin_DIR_POMPE_WASTE = 34;
const int Pin_EN_POMPE_WASTE = 30;  
int volume_Jour_POMPE_WASTE = 0 ;// volume à ajouter par jour 
int Volume_ajoute_POMPE_WASTE = 0;
int Volume_force_POMPE_WASTE = 0;
float Consigne_nb_pas_POMPE_WASTE= 0;
float nb_pas_POMPE_WASTE = 0;
float Conv_mL_Pas_POMPE_WASTE = 42650; //2*422017/20  426609 demi pas/20mL en 60s. 20mL de liquide mesuré pour 60s de débit
unsigned long TimeStep_POMPE_WASTE = 0; //pour se souvenir du dernier micro pas, ne pas toucher
unsigned long periode_Step_POMPE_WASTE = 50; //temps entre deux micro pas, defini le debit
boolean sortie_POMPE_WASTE = HIGH; //gère l'état du step, ne pas toucher


const int Pin_STEP_Agitateur = 8;

//vanne 1 (Z)
const int Pin_STEP_Vanne = 46; //A0;
const int Pin_DIR_Vanne = 48; //A1;
const int Pin_EN_Vanne = A8; //38; 
const int Pin_Endstop_VANNE = 32;

//POUSSE SERINGUE (X)
const int Pin_STEP_PS = A0; //A6;
const int Pin_DIR_PS = A1; //A7;
const int Pin_EN_PS = 38; //A2; 
const int Pin_Endstop_PS_MIN = 2;
const int Pin_Endstop_PS_MAX = 3;

byte PS_DEPART = 0xB0; //non initialisé
byte PS_INIT = 0xB1; //Encours
byte PS_ENCOURS = 0xB2; //Encours
byte PS_POS_MIN = 0xB3; // en position MIN
byte PS_POS_MAX = 0xB4; // en position MAX
float Conversion_vol = 381 ; //649.11; //1298.227; // micropas/uL
unsigned long Timeout_PS = 20000; //timeou de déplacement de la vanne
unsigned long Time_PS = 0; //temps ecoulé

const int Pin_BULLE_HCL = A9;

// PSINJ 200 3
// PSASP 200 3
byte Vanne_DEPART = 0xA0; //non initialisé
byte Vanne_INIT = 0xA1; //Encours
byte Vanne_ENCOURS = 0xA2; //Encours
byte Vanne_WAST = 0xA3; //Poubelle
byte Vanne_ECHANTILLON = 0xA4; //ECHANTILLON
byte Vanne_HCL = 0xA5; //recipient HCL
byte Vanne_ERROR = 0xA6; //recipient HCL

byte ETAT_VANNE = Vanne_DEPART ; //Etat de la vanne

float Position_WAST = 115000 ; //relatif par rapport à l'init
float Position_ECHANTILLON = 102400 ; //relatif par rapport à la position wast = 90°
float Position_HCL = 102400 ; //relatif par rapport à la position Echantillon = 90°

float STEP_TOUR = 409600;//nombre de step/tour  (niveau haut + bas)

unsigned long Timeout_Vanne = 10000; //timeou de déplacement de la vanne
unsigned long Time_Vanne = 0; //temps ecoulé




//TITRATION
float Valeur_PH = 0;
float Valeur_KH = 0;
float Volume_Milieu = 25; //Volume de l'eau de l'aquarium
float Concentration = 0.1; //N molaire d'acide HCL
float Facteur = 2.8; //Facteur relatif à l'alcalinite

int Volume_Tuyau = 7 ;


String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void serialEvent();
void Decode_Trame();
String getValue(String data, char separator, int index);
void VANNE_GO_LEFT(float commande);
void VANNE_GO_RIGHT(float commande);
void VANNE_GO_INIT();
void VANNE_GO_WAST();
void VANNE_GO_ECHANTILLON();
void VANNE_GO_HCL();

void PS_GO_MIN();
void PS_GO_MAX();
void PS_INJECTER(float volume, int PS_speed);
void PS_ASPIRER(float volume, int PS_speed);
void PS_PRIME();
void BUBULLE();

void INJECT_AQUA(float volume);
void ASPIRE_AQUA(float volume);
void INJECT_WASTE(float volume);
void ASPIRE_WASTE(float volume);
void SET_AGITATEUR(int vitesse);

float PH_Commande(String commande);

void Titration1();
void Titration();
void LOAD_PS();
void attente(unsigned long duree);
void TEST();

void Envoi_Etat();
void Envoi_Commande(byte ID=0x000,byte commande0=0x00,byte commande1=0x00,byte commande2=0x00,byte commande3=0x00,byte commande4=0x00,byte commande5=0x00,byte commande6=0x00,byte commande7=0x00);
void ASK(int numero);

void setup() {
  // put your setup code here, to run once:
  pinMode(Pin_STEP_POMPE_AQUA, OUTPUT); // step
  pinMode(Pin_DIR_POMPE_AQUA, OUTPUT); // dir
  pinMode(Pin_EN_POMPE_AQUA, OUTPUT); // enable
  digitalWrite(Pin_DIR_POMPE_AQUA, LOW);
  digitalWrite(Pin_EN_POMPE_AQUA, HIGH);

  pinMode(Pin_STEP_POMPE_WASTE, OUTPUT); // step
  pinMode(Pin_DIR_POMPE_WASTE, OUTPUT); // dir
  pinMode(Pin_EN_POMPE_WASTE, OUTPUT); // enable
  digitalWrite(Pin_DIR_POMPE_WASTE, LOW);
  digitalWrite(Pin_EN_POMPE_WASTE, HIGH);
  
  pinMode(Pin_STEP_Vanne, OUTPUT); // step
  pinMode(Pin_DIR_Vanne, OUTPUT); // dir
  pinMode(Pin_EN_Vanne, OUTPUT); // enable
  pinMode (Pin_Endstop_VANNE, INPUT);
  pinMode(Pin_STEP_PS, OUTPUT); // step
  pinMode(Pin_DIR_PS, OUTPUT); // dir
  pinMode(Pin_EN_PS, OUTPUT); // enable
  pinMode (Pin_Endstop_PS_MIN, INPUT);
  pinMode (Pin_Endstop_PS_MAX, INPUT);

  digitalWrite(Pin_EN_PS, HIGH);
  digitalWrite(Pin_EN_Vanne, HIGH);
  
  pinMode(Pin_STEP_Agitateur, OUTPUT);
  digitalWrite(Pin_STEP_Agitateur, LOW);

  
  Serial.begin(115200);
  Serial1.begin(9600); 

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
  Time_Send = millis();


  //Start RTC
  Wire.begin();
  for (int i = 0; i < 6; i++) { 
    EEPROM.get(adrH1[i], H1[i]);
    EEPROM.get(adrM1[i], M1[i]);
  }
  ETAT = ETAT_OK; 
  dOW = int(myRTC.getDoW());
  hour = int(myRTC.getHour(h12Flag, pmFlag));
  minute = int(myRTC.getMinute());

  PH_Commande("Sleep");
  
}
 
void loop() {
  //PS_PRIME();
  //VANNE_GO_RIGHT(1000000);
  //Serial.println(analogRead(Pin_BULLE_HCL));
  //Serial.println(digitalRead(Pin_Endstop_PS_MIN));
  Envoi_ETAT();
  ECOUTE_Bus_CAN();
  
  serialEvent();
  if (stringComplete) {
    Decode_Trame();
  }

  if (hour == 12 && minute == 0 && ETAT == ETAT_OK ){
            Titration();
       }  

  
}

void Envoi_ETAT(){
  if (millis()- Time_Send >= 1000){ //boucle toutes les x ms
      Time_Send = millis();
      dOW = int(myRTC.getDoW());
      hour = int(myRTC.getHour(h12Flag, pmFlag));
      minute = int(myRTC.getMinute());
      second = int(myRTC.getSecond());
      float mesure = 0;
      byte code = 0xA5;
      if (retour == true) {
        mesure = Valeur_PH;
        code = 0xA5;
        retour = false;
      }
      else{
        mesure = Valeur_KH;
        code = 0xA6;
        retour = true;
      }

      int mesure_hi = int(mesure);
      float buffer1 = (mesure - mesure_hi)*100;
      int mesure_low = int(buffer1) ;
      Envoi_Commande(ID,ETAT,code,hour,minute,second,dOW,mesure_hi,mesure_low); 
      }
}

float ECOUTE_Bus_CAN(){
          if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
            if (canMsg.can_id == ID){
              Serial.println("reception");  
               if (canMsg.data[0] == ASK_CONFIG) {
                  Serial.println("ASK");
                  ASK(canMsg.data[1]);
               }
               if (canMsg.data[0] == TITRATION) {
                  Serial.println("demande de TITRATION");
                  Titration();
               }
               if (canMsg.data[0] == PRIME) {
                  Serial.println("demande de PRIME");
                  PS_PRIME();
               }
               if (canMsg.data[0] == SET) {
                  Serial.println("SET");
                  int num = 0;
                  if (canMsg.data[2] == SET_HORAIRE_t1){num = 0;}
                  else if (canMsg.data[2] == SET_HORAIRE_t2){num = 1;}
                  else if (canMsg.data[2] == SET_HORAIRE_t3){num = 2;}
                  else if (canMsg.data[2] == SET_HORAIRE_t4){num = 3;}
                  else if (canMsg.data[2] == SET_HORAIRE_t5){num = 4;}
                  else if (canMsg.data[2] == SET_HORAIRE_t6){num = 5;}
                  H1[num] = canMsg.data[4];
                  M1[num] = canMsg.data[5];
                  EEPROM.put(adrH1[num], H1[num]);
                  EEPROM.put(adrM1[num], M1[num]);             
                  }
               if (canMsg.data[0] == SET_HOUR) {
                  int hour_set = int(canMsg.data[1]); 
                  int minute_set = int(canMsg.data[2]); 
                  myRTC.setClockMode(false);  // set to 24h
                  myRTC.setHour(hour_set);
                  myRTC.setMinute(minute_set);
                  myRTC.setSecond(55);
        
                 }
               if (canMsg.data[0] == SET_DAY) {
                  int day = int(canMsg.data[1]); 
                  myRTC.setDoW(day);
                 }
            }
        }
}

// pour l'USB
void Decode_Trame(){ 
    if (getValue(inputString,' ',0)=="l"){VANNE_GO_LEFT(5000);}
    if (getValue(inputString,' ',0)=="r"){VANNE_GO_RIGHT(5000);}
    if (getValue(inputString,' ',0)=="init"){VANNE_GO_INIT();}
    if (getValue(inputString,' ',0)=="wast"){VANNE_GO_WAST();}
    if (getValue(inputString,' ',0)=="echan"){VANNE_GO_ECHANTILLON();}
    if (getValue(inputString,' ',0)=="HCL"){VANNE_GO_HCL();}
    if (getValue(inputString,' ',0)=="PSMIN"){PS_GO_MIN();}
    if (getValue(inputString,' ',0)=="PSMAX"){PS_GO_MAX();}
    if (getValue(inputString,' ',0)=="PRIME"){PS_PRIME();}
    if (getValue(inputString,' ',0)=="STOP"){Flag_STOP = true;}
    if (getValue(inputString,' ',0)=="TITRATION"){Titration();}
    if (getValue(inputString,' ',0)=="TEST"){TEST();}
    if (getValue(inputString,' ',0)=="PSINJ"){
      float com = (getValue(inputString,' ',1)).toInt();
      float spe = (getValue(inputString,' ',2)).toInt();
      PS_INJECTER(com,spe);
    }
    if (getValue(inputString,' ',0)=="PSASP"){
      float com = (getValue(inputString,' ',1)).toInt();
      float spe = (getValue(inputString,' ',2)).toInt();
      PS_ASPIRER(com,spe);
    }
    if (getValue(inputString,' ',0)=="AQUAINJ"){
      float com = (getValue(inputString,' ',1)).toInt();
      //float spe = (getValue(inputString,' ',2)).toInt();
      INJECT_AQUA(com);
    }
    if (getValue(inputString,' ',0)=="AQUAASP"){
      float com = (getValue(inputString,' ',1)).toInt();
      //float spe = (getValue(inputString,' ',2)).toInt();
      ASPIRE_AQUA(com);
    }
    if (getValue(inputString,' ',0)=="WASTEINJ"){
      float com = (getValue(inputString,' ',1)).toInt();
      //float spe = (getValue(inputString,' ',2)).toInt();
      INJECT_WASTE(com);
    }
    if (getValue(inputString,' ',0)=="WASTEASP"){
      float com = (getValue(inputString,' ',1)).toInt();
      //float spe = (getValue(inputString,' ',2)).toInt();
      ASPIRE_WASTE(com);
    }
    if (getValue(inputString,' ',0)=="AGI"){
      float vit = (getValue(inputString,' ',1)).toInt();
      //float spe = (getValue(inputString,' ',2)).toInt();
      SET_AGITATEUR(vit);
    }
    if (getValue(inputString,' ',0)=="PH"){
      String commande1 = (getValue(inputString,' ',1));
      PH_Commande(commande1);
    }

    inputString = "";
    stringComplete = false;
    
  }

void TEST(){
  unsigned long Time_test = millis();
  Flag_STOP = false;
  float pH_mesure = 10;
  while (Flag_STOP == false){
    Serial.print(millis()-Time_test);
    Serial.print("\t");
    pH_mesure = PH_Commande("R");
    attente(2);    
  }
}

void Titration1(){ // pour test
  ETAT = ENCOURS;
  Valeur_KH = 0;
  Envoi_ETAT();
  attente(20);
  Valeur_KH = 8.15;
  ETAT = ETAT_OK;
}

void Titration(){
  ETAT = ENCOURS;
  PS_PRIME();
  Envoi_ETAT();
  Serial.println("Début Titration");
  Flag_STOP = false;
  COMMENTAIRE = false;
  float pH_mesure = 10;
  float Volume_injecte = 0;
  float Volume_temporaire = 0; //pour reload la seringue avant d'arrivé au MIN et fausser le volume injecté
  float vol = 100;
  attente(30);
  pH_mesure = PH_Commande("R");
  pH_mesure = PH_Commande("R");
  Valeur_PH = pH_mesure;
  Serial.print(0); 
  Serial.print("\t");
  Serial.println(pH_mesure);

  while (pH_mesure > 6.5 and Flag_STOP == false){
    vol = 100;
    if (Volume_temporaire > 400){ 
      LOAD_PS();
      Volume_temporaire = 0;
      }
    PS_INJECTER(vol,20);
    Volume_temporaire = Volume_temporaire + vol;
    Volume_injecte = Volume_injecte + vol;
    attente(20);
    pH_mesure = PH_Commande("R");
    pH_mesure = PH_Commande("R");
    Serial.print(Volume_injecte); 
    Serial.print("\t");
    Serial.println(pH_mesure); 
  }

  while (pH_mesure > 5.5 and Flag_STOP == false){
      vol = 50;
      if (Volume_temporaire > 400){ 
        LOAD_PS();
        Volume_temporaire = 0;
        }
      PS_INJECTER(vol,20);
      Volume_temporaire = Volume_temporaire + vol;
      Volume_injecte = Volume_injecte + vol;
      attente(20);
      pH_mesure = PH_Commande("R");
      pH_mesure = PH_Commande("R");
      Serial.print(Volume_injecte); 
      Serial.print("\t");
      Serial.println(pH_mesure); 
  }

  while (pH_mesure > 5 and Flag_STOP == false){
      vol = 10;
      if (Volume_temporaire > 400){ 
        LOAD_PS();
        Volume_temporaire = 0;
        }
      PS_INJECTER(vol,20);
      Volume_temporaire = Volume_temporaire + vol;
      Volume_injecte = Volume_injecte + vol;
      attente(20);
      pH_mesure = PH_Commande("R");
      pH_mesure = PH_Commande("R");
      Serial.print(Volume_injecte); 
      Serial.print("\t");
      Serial.println(pH_mesure); 
  }
  while (pH_mesure > 4.5 and Flag_STOP == false){  //4.5 limite qui permet de trouver le KH
      vol = 5;
      if (Volume_temporaire > 400){ 
        LOAD_PS();
        Volume_temporaire = 0;
        }
      PS_INJECTER(vol,20);
      Volume_temporaire = Volume_temporaire + vol;
      Volume_injecte = Volume_injecte + vol;
      attente(20);
      pH_mesure = PH_Commande("R");
      pH_mesure = PH_Commande("R");
      Serial.print(Volume_injecte); 
      Serial.print("\t");
      Serial.println(pH_mesure); 
  }
  
  Valeur_KH = (Volume_injecte/1000)*Concentration*Facteur / (Volume_Milieu/1000);

  while (pH_mesure > 4 and Flag_STOP == false){  //pour continuer à avoir des points
      vol = 5;
      if (Volume_temporaire > 400){ 
        LOAD_PS();
        Volume_temporaire = 0;
        }
      PS_INJECTER(vol,20);
      Volume_temporaire = Volume_temporaire + vol;
      Volume_injecte = Volume_injecte + vol;
      attente(20);
      pH_mesure = PH_Commande("R");
      pH_mesure = PH_Commande("R");
      Serial.print(Volume_injecte); 
      Serial.print("\t");
      Serial.println(pH_mesure); 
  }
  
  INJECT_WASTE(27);
  INJECT_AQUA(25);
  attente(10);
  SET_AGITATEUR(0);
  Serial.println("FIN TIRTATION"); 
  COMMENTAIRE = true;
  PH_Commande("Sleep");
  ETAT = ETAT_OK;
}

void LOAD_PS(){
  VANNE_GO_HCL();
  PS_GO_MAX();
  PS_INJECTER(10,50); //ratrappage de jeu éventuel
  VANNE_GO_ECHANTILLON();
}

void attente(unsigned long duree){
  inputString = "";
  stringComplete = false;
  unsigned long Time_attente = millis();
  while(millis()-Time_attente < duree*1000){
    Envoi_ETAT();
    serialEvent();
    if (stringComplete) {
      Decode_Trame();
    }
  }
}

float PH_Commande(String commande){
    sensorstring = "";
    Serial1.print(commande);                       
    Serial1.print('\r');
    boolean Flag_reception = false;
    unsigned long Time_PH = millis();
    while(millis()-Time_PH < 1000 and Flag_reception == false){// and Serial1.available() == false){ 
        //Serial.println("attente");
        if (Serial1.available()) {               //if a string from the Atlas Scientific product has been received in its entirety
          sensorstring = Serial1.readStringUntil(13);         //read the string until we see a <CR>
          //Serial.println(sensorstring);                     //send that string to the PC's serial monitor
                                                       
          if (isdigit(sensorstring[0])) {//if the first character in the string is a digit
            Flag_reception = true;
            pH = sensorstring.toFloat();                    //convert the string to a floating point number so it can be evaluated by the Arduino         
            if (COMMENTAIRE== true){
              Serial.println(pH); 
            }                      
            }
      }
      //sensorstring = "";
    }
    //sensorstring = "";
    /*
    if (commande == "R"){
      return pH;
    }
    else{return 0;}
    */
    return pH;
}

void PS_PRIME(){
  Serial.println("debut PRIME");

  ASPIRE_AQUA(Volume_Tuyau+2); //vide le tuyau d'alimentation
  INJECT_AQUA(Volume_Tuyau); //rempli le tuyau d'alimentation
  
  INJECT_WASTE(30);
  INJECT_AQUA(23);
  SET_AGITATEUR(200);
 
  VANNE_GO_INIT();
  VANNE_GO_WAST();
  PS_GO_MIN();
  VANNE_GO_HCL();
  PS_GO_MAX();
  PS_INJECTER(50,1);

  PS_INJECTER(50,2);
  PS_ASPIRER(50,2);
  PS_INJECTER(50,2);
  PS_ASPIRER(50,2);
  PS_INJECTER(50,2);
  PS_ASPIRER(50,2);
  PS_INJECTER(50,2);
  PS_ASPIRER(50,2);

  VANNE_GO_WAST();
  PS_GO_MIN();
  VANNE_GO_HCL();
  PS_ASPIRER(100,2);
  VANNE_GO_ECHANTILLON();
  PS_GO_MAX();
  VANNE_GO_WAST();
  PS_GO_MIN();
  VANNE_GO_HCL();
  PS_GO_MAX();  
  VANNE_GO_ECHANTILLON();
  PS_INJECTER(222,2); //A VERIFIER 162
  /*
  BUBULLE();
  Serial.println("FIN");
  */
  INJECT_WASTE(27);
  INJECT_AQUA(25);
  LOAD_PS();
  
}


void SET_AGITATEUR(int vitesse){
  if (vitesse < 110 and vitesse !=0){vitesse=110;}
  if (vitesse > 255){vitesse=255;}
  analogWrite(Pin_STEP_Agitateur,vitesse);
}

void INJECT_WASTE(float volume){
      float Consigne_nb_pas_POMPE_WASTE_bis = volume*Conv_mL_Pas_POMPE_WASTE/2; //  divisé par deux car on gere un niveau haut et bas
      if (COMMENTAIRE== true){
        Serial.print("INJECTER  ");
        Serial.println(Consigne_nb_pas_POMPE_WASTE_bis); 
      }
      periode_Step_POMPE_WASTE = 30;
      //ETAT_pompe_POMPE_AQUA = ENCOURS;
      digitalWrite(Pin_EN_POMPE_WASTE, LOW);
      digitalWrite(Pin_DIR_POMPE_WASTE, LOW);
      float count1 = 0;
      for (count1; count1<=Consigne_nb_pas_POMPE_WASTE_bis;count1++) {
        digitalWrite(Pin_STEP_POMPE_WASTE, HIGH);
        delayMicroseconds(periode_Step_POMPE_WASTE);//min 2000  max 50
        digitalWrite(Pin_STEP_POMPE_WASTE, LOW);
        delayMicroseconds(periode_Step_POMPE_WASTE);
      }
      digitalWrite(Pin_EN_POMPE_WASTE, HIGH);
      //ETAT_pompe_POMPE_AQUA = DEPART; 
}

void ASPIRE_WASTE(float volume){
      float Consigne_nb_pas_POMPE_WASTE_bis = volume*Conv_mL_Pas_POMPE_WASTE/2; //  divisé par deux car on gere un niveau haut et bas
      if (COMMENTAIRE== true){
        Serial.print("ASPIRER  ");
        Serial.println(Consigne_nb_pas_POMPE_WASTE_bis);
      }
      periode_Step_POMPE_WASTE = 30;
      //ETAT_pompe_POMPE_AQUA = ENCOURS;
      digitalWrite(Pin_EN_POMPE_WASTE, LOW);
      digitalWrite(Pin_DIR_POMPE_WASTE, HIGH);
      float count2 = 0;
      for (count2; count2<=Consigne_nb_pas_POMPE_WASTE_bis;count2++) {
        digitalWrite(Pin_STEP_POMPE_WASTE, HIGH);
        delayMicroseconds(periode_Step_POMPE_WASTE);//min 2000  max 50
        digitalWrite(Pin_STEP_POMPE_WASTE, LOW);
        delayMicroseconds(periode_Step_POMPE_WASTE);
      }
      digitalWrite(Pin_EN_POMPE_WASTE, HIGH);
      //ETAT_pompe_POMPE_AQUA = DEPART; 
}
void INJECT_AQUA(float volume){
      float Consigne_nb_pas_POMPE_AQUA_bis = volume*Conv_mL_Pas_POMPE_AQUA/2; //  divisé par deux car on gere un niveau haut et bas
      if (COMMENTAIRE== true){
        Serial.print("INJECTER  ");
        Serial.println(Consigne_nb_pas_POMPE_AQUA_bis);
      }
      periode_Step_POMPE_AQUA = 30;
      //ETAT_pompe_POMPE_AQUA = ENCOURS;
      digitalWrite(Pin_EN_POMPE_AQUA, LOW);
      digitalWrite(Pin_DIR_POMPE_AQUA, LOW);
      float count1 = 0;
      for (count1; count1<=Consigne_nb_pas_POMPE_AQUA_bis;count1++) {
        digitalWrite(Pin_STEP_POMPE_AQUA, HIGH);
        delayMicroseconds(periode_Step_POMPE_AQUA);//min 2000  max 50
        digitalWrite(Pin_STEP_POMPE_AQUA, LOW);
        delayMicroseconds(periode_Step_POMPE_AQUA);
      }
      digitalWrite(Pin_EN_POMPE_AQUA, HIGH);
      //ETAT_pompe_POMPE_AQUA = DEPART; 
}

void ASPIRE_AQUA(float volume){
      float Consigne_nb_pas_POMPE_AQUA_bis = volume*Conv_mL_Pas_POMPE_AQUA/2; //  divisé par deux car on gere un niveau haut et bas
      if (COMMENTAIRE== true){
        Serial.print("ASPIRER  ");
        Serial.println(Consigne_nb_pas_POMPE_AQUA_bis);
      }
      periode_Step_POMPE_AQUA = 30;
      //ETAT_pompe_POMPE_AQUA = ENCOURS;
      digitalWrite(Pin_EN_POMPE_AQUA, LOW);
      digitalWrite(Pin_DIR_POMPE_AQUA, HIGH);
      float count2 = 0;
      for (count2; count2<=Consigne_nb_pas_POMPE_AQUA_bis;count2++) {
        digitalWrite(Pin_STEP_POMPE_AQUA, HIGH);
        delayMicroseconds(periode_Step_POMPE_AQUA);//min 2000  max 50
        digitalWrite(Pin_STEP_POMPE_AQUA, LOW);
        delayMicroseconds(periode_Step_POMPE_AQUA);
      }
      digitalWrite(Pin_EN_POMPE_AQUA, HIGH);
      //ETAT_pompe_POMPE_AQUA = DEPART; 
}


void BUBULLE(){
      digitalWrite(Pin_EN_PS, LOW);
      digitalWrite(Pin_DIR_PS, LOW);
      float count = 0;
      Time_PS = millis();
      float commande = 0;
      commande = 20 * Conversion_vol;
      while(count <= commande and millis()-Time_PS < Timeout_PS and analogRead(Pin_BULLE_HCL)<100 and digitalRead(Pin_Endstop_PS_MIN) == false){ 
            count ++;
            digitalWrite(Pin_STEP_PS, HIGH);
            delayMicroseconds(500);//max speed 10
            digitalWrite(Pin_STEP_PS, LOW);
            delayMicroseconds(500);
          }
       digitalWrite(Pin_EN_PS, HIGH);
       float mesure = count/Conversion_vol;
       Serial.print("BULLE à ");
       Serial.println(mesure);
}

void PS_GO_MIN(){
      if (COMMENTAIRE== true){
        Serial.println("Recherche MIN");
        }
      digitalWrite(Pin_EN_PS, LOW);
      digitalWrite(Pin_DIR_PS, LOW);
      float count = 0;
      Time_PS = millis();
      while(millis()-Time_PS < Timeout_PS and digitalRead(Pin_Endstop_PS_MIN) == false){ 
            count ++;
            digitalWrite(Pin_STEP_PS, HIGH);
            delayMicroseconds(5);//max speed 10
            digitalWrite(Pin_STEP_PS, LOW);
            delayMicroseconds(5);
          }    
       digitalWrite(Pin_EN_PS, HIGH);
       Serial.print("MIN à ");
       Serial.println(count);
       
}
void PS_GO_MAX(){
      if (COMMENTAIRE== true){
        Serial.println("Recherche MAX");
        }
      digitalWrite(Pin_EN_PS, LOW);
      digitalWrite(Pin_DIR_PS, HIGH);
      float count = 0;
      Time_PS = millis();
      while(millis()-Time_PS < Timeout_PS and digitalRead(Pin_Endstop_PS_MAX) == false){ 
            count ++;
            digitalWrite(Pin_STEP_PS, HIGH);
            delayMicroseconds(5);//max speed 10
            digitalWrite(Pin_STEP_PS, LOW);
            delayMicroseconds(5);
          }    
       digitalWrite(Pin_EN_PS, HIGH);
       if (COMMENTAIRE== true){
          Serial.print("MAX à ");
          Serial.println(count);
         }
}

void PS_ASPIRER(float volume,int PS_speed = 10){
      if (COMMENTAIRE== true){
        Serial.println("Aspirer");
      }
      digitalWrite(Pin_EN_PS, LOW);
      digitalWrite(Pin_DIR_PS, HIGH);
      float count = 0;
      float commande = 0;
      Time_PS = millis();
      commande = volume * Conversion_vol;
      while(count <= commande and millis()-Time_PS < Timeout_PS and digitalRead(Pin_Endstop_PS_MAX) == false){ 
            count ++;
            digitalWrite(Pin_STEP_PS, HIGH);
            delayMicroseconds(PS_speed);//max speed 10
            digitalWrite(Pin_STEP_PS, LOW);
            delayMicroseconds(PS_speed);
          }    
       digitalWrite(Pin_EN_PS, HIGH);
       if (COMMENTAIRE== true){
        Serial.println("FIN ASPIRER ");
       }
}

void PS_INJECTER(float volume,int PS_speed = 10){
      if (COMMENTAIRE== true){
        Serial.println("Injecter");
      }
      digitalWrite(Pin_EN_PS, LOW);
      digitalWrite(Pin_DIR_PS, LOW);
      float count = 0;
      Time_PS = millis();
      float commande = 0;
      commande = volume * Conversion_vol;
      while(count <= commande and millis()-Time_PS < Timeout_PS and digitalRead(Pin_Endstop_PS_MIN) == false){ 
            count ++;
            digitalWrite(Pin_STEP_PS, HIGH);
            delayMicroseconds(PS_speed);//max speed 10
            digitalWrite(Pin_STEP_PS, LOW);
            delayMicroseconds(PS_speed);
          }    
       digitalWrite(Pin_EN_PS, HIGH);
       if (COMMENTAIRE== true){
        Serial.println("FIN INJECTER ");
       }
}

 
  /*
  VANNE_GO_INIT();
  delay(2000);
  VANNE_GO_WAST();
  delay(2000);
  VANNE_GO_ECHANTILLON();
  delay(2000);
  VANNE_GO_HCL();
  delay(2000);
  VANNE_GO_ECHANTILLON();
  delay(2000);
  VANNE_GO_WAST();
  delay(60000);
  */
  /*
  int count1 =0;
  for (count1; count1<=10000;count1++){
    
          Serial.println(count1);
          digitalWrite(Pin_EN_1, LOW);
          digitalWrite(Pin_DIR_1, HIGH);
          digitalWrite(Pin_STEP_1, HIGH);
          float count = 0;
          for (count; count<=409600;count++) { //204800*2/tour    backlash <5000  90° = 102400
            digitalWrite(Pin_STEP_1, HIGH);
            delayMicroseconds(10);//min 2000  max 50
            digitalWrite(Pin_STEP_1, LOW);
            delayMicroseconds(10);
          }
          digitalWrite(Pin_EN_1, HIGH);
         delay(1000); 

          digitalWrite(Pin_EN_1, LOW);
          digitalWrite(Pin_DIR_1, LOW);
          digitalWrite(Pin_STEP_1, HIGH);
          count = 0;
          for (count; count<=409600;count++) { //204800*2/tour    backlash <5000  90° = 102400
            digitalWrite(Pin_STEP_1, HIGH);
            delayMicroseconds(10);//min 2000  max 50
            digitalWrite(Pin_STEP_1, LOW);
            delayMicroseconds(10);
          }
          digitalWrite(Pin_EN_1, HIGH);
         delay(1000); 
  }

     */
     /*
      digitalWrite(Pin_EN_1, LOW);
      digitalWrite(Pin_DIR_1, LOW);
      digitalWrite(Pin_STEP_1, HIGH);
      count = 0;
      for (count; count<=204800;count++) { //204800*2/tour    backlash <5000  90° = 102400
        digitalWrite(Pin_STEP_1, HIGH);
        delayMicroseconds(10);//min 2000  max 50
        digitalWrite(Pin_STEP_1, LOW);
        delayMicroseconds(10);
      }
      digitalWrite(Pin_EN_1, HIGH);
      */
   


void VANNE_GO_LEFT(float commande){
      if (COMMENTAIRE== true){
        Serial.println("go to left");
      }
      digitalWrite(Pin_EN_Vanne, LOW);
      digitalWrite(Pin_DIR_Vanne, HIGH);
      float count = 0;
      Time_Vanne = millis();
      while(count <= commande and millis()-Time_Vanne < Timeout_Vanne){ 
            count ++;
            digitalWrite(Pin_STEP_Vanne, HIGH);
            delayMicroseconds(10);//max speed 10
            digitalWrite(Pin_STEP_Vanne, LOW);
            delayMicroseconds(10);
          }    
       digitalWrite(Pin_EN_Vanne, HIGH);
}

void VANNE_GO_RIGHT(float commande){
      if (COMMENTAIRE== true){
        Serial.println("go to right");
      }
      digitalWrite(Pin_EN_Vanne, LOW);
      digitalWrite(Pin_DIR_Vanne, LOW);
      float count = 0;
      Time_Vanne = millis();
      while(count <= commande and millis()-Time_Vanne < Timeout_Vanne){ 
            count ++;
            digitalWrite(Pin_STEP_Vanne, HIGH);
            delayMicroseconds(10);//max speed 10
            digitalWrite(Pin_STEP_Vanne, LOW);
            delayMicroseconds(10);
          }    
       digitalWrite(Pin_EN_Vanne, HIGH);
}


void VANNE_GO_INIT(){
      ETAT_VANNE = Vanne_ENCOURS;
      digitalWrite(Pin_EN_Vanne, LOW);
      digitalWrite(Pin_DIR_Vanne, HIGH);
      
      float count = 0;
      if (digitalRead(Pin_Endstop_VANNE) ==0){ // dégagement du endstop
          digitalWrite(Pin_DIR_Vanne, LOW);
          Time_Vanne = millis();
          float commande = 20000;
          while(count <= commande and millis()-Time_Vanne < Timeout_Vanne ){ //digitalRead(Pin_Endstop_VANNE) ==1 and 
                count ++;
                digitalWrite(Pin_STEP_Vanne, HIGH);
                delayMicroseconds(10);//max speed 10
                digitalWrite(Pin_STEP_Vanne, LOW);
                delayMicroseconds(10);
              }  
             digitalWrite(Pin_DIR_Vanne, HIGH);     
        }
      count = 0;
      while(digitalRead(Pin_Endstop_VANNE) ==1 and count <= STEP_TOUR ){ 
            count ++;
            digitalWrite(Pin_STEP_Vanne, HIGH);
            delayMicroseconds(10);//max speed 10
            digitalWrite(Pin_STEP_Vanne, LOW);
            delayMicroseconds(10);
          }    
       digitalWrite(Pin_EN_Vanne, HIGH);
       if (count<STEP_TOUR){ETAT_VANNE = Vanne_INIT;}
       else{ETAT_VANNE = Vanne_ERROR;}
       
       if (COMMENTAIRE== true){
        Serial.print("ETAT_VANNE ");
        Serial.println(ETAT_VANNE);
       }
}

void VANNE_GO_WAST(){
      if (COMMENTAIRE== true){
        Serial.println("go to wast");
      }
      float commande = 0;
      boolean sens = LOW;
      if (ETAT_VANNE == Vanne_INIT){
        commande = Position_WAST;
        sens = LOW;
      }
      else if (ETAT_VANNE == Vanne_WAST){
        return;
      }
      if (ETAT_VANNE == Vanne_ECHANTILLON){
        commande = Position_ECHANTILLON;
        sens = HIGH;
      }
      if (ETAT_VANNE == Vanne_HCL){
        commande = Position_ECHANTILLON + Position_HCL ;
        sens = HIGH;
      }
      ETAT_VANNE = Vanne_ENCOURS;
      digitalWrite(Pin_EN_Vanne, LOW);
      digitalWrite(Pin_DIR_Vanne, sens);
      float count = 0;
      Time_Vanne = millis();
      if (COMMENTAIRE== true){
        Serial.println(commande);
      }
      while(count <= commande and millis()-Time_Vanne < Timeout_Vanne ){ //digitalRead(Pin_Endstop_VANNE) ==1 and 
            count ++;
            digitalWrite(Pin_STEP_Vanne, HIGH);
            delayMicroseconds(10);//max speed 10
            digitalWrite(Pin_STEP_Vanne, LOW);
            delayMicroseconds(10);
          }    
       digitalWrite(Pin_EN_Vanne, HIGH);
       if (millis()-Time_Vanne < Timeout_Vanne and digitalRead(Pin_Endstop_VANNE) ==1){
          ETAT_VANNE = Vanne_WAST ;}
       else{ETAT_VANNE = Vanne_ERROR;}
       if (COMMENTAIRE== true){
         Serial.print("ETAT_VANNE ");
         Serial.println(ETAT_VANNE);
       }
}

void VANNE_GO_ECHANTILLON(){
      if (COMMENTAIRE== true){
        Serial.println("go to Echantillon");
      }
      float commande = 0;
      boolean sens = LOW;
      if (ETAT_VANNE == Vanne_INIT){
        commande = Position_WAST + Position_ECHANTILLON ;
        sens = LOW;
      }
      else if (ETAT_VANNE == Vanne_WAST){
        commande = Position_ECHANTILLON;
        sens = LOW;
      }
      if (ETAT_VANNE == Vanne_ECHANTILLON){
        return;
      }
      if (ETAT_VANNE == Vanne_HCL){
        commande = Position_HCL ;
        sens = HIGH;
      }
      ETAT_VANNE = Vanne_ENCOURS;
      digitalWrite(Pin_EN_Vanne, LOW);
      digitalWrite(Pin_DIR_Vanne, sens);
      float count = 0;
      Time_Vanne = millis();
      if (COMMENTAIRE== true){
        Serial.println(commande);
      }
      while(count <= commande and millis()-Time_Vanne < Timeout_Vanne ){ //digitalRead(Pin_Endstop_VANNE) ==1 and 
            count ++;
            digitalWrite(Pin_STEP_Vanne, HIGH);
            delayMicroseconds(10);//max speed 10
            digitalWrite(Pin_STEP_Vanne, LOW);
            delayMicroseconds(10);
          }    
       digitalWrite(Pin_EN_Vanne, HIGH);
       if (millis()-Time_Vanne < Timeout_Vanne and digitalRead(Pin_Endstop_VANNE) ==1){
          ETAT_VANNE = Vanne_ECHANTILLON ;}
       else{ETAT_VANNE = Vanne_ERROR;}
       if (COMMENTAIRE== true){
         Serial.print("ETAT_VANNE ");
         Serial.println(ETAT_VANNE);
       }
}

void VANNE_GO_HCL(){
      if (COMMENTAIRE== true){
        Serial.println("go to HCL");
      }
      float commande = 0;
      boolean sens = LOW;
      if (ETAT_VANNE == Vanne_INIT){
        commande = Position_WAST + Position_ECHANTILLON +Position_HCL;
        sens = LOW;
      }
      else if (ETAT_VANNE == Vanne_WAST){
        commande = Position_ECHANTILLON + Position_HCL;
        sens = LOW;
      }
      else if (ETAT_VANNE == Vanne_ECHANTILLON){
        commande = Position_HCL;
        sens = LOW;
      }
      if (ETAT_VANNE == Vanne_HCL){
        return;
      }
      ETAT_VANNE = Vanne_ENCOURS;
      digitalWrite(Pin_EN_Vanne, LOW);
      digitalWrite(Pin_DIR_Vanne, sens);
      float count = 0;
      Time_Vanne = millis();
      if (COMMENTAIRE== true){
        Serial.println(commande);
      }
      while(count <= commande and millis()-Time_Vanne < Timeout_Vanne ){ //digitalRead(Pin_Endstop_VANNE) ==1 and 
            count ++;
            digitalWrite(Pin_STEP_Vanne, HIGH);
            delayMicroseconds(10);//max speed 10
            digitalWrite(Pin_STEP_Vanne, LOW);
            delayMicroseconds(10);
          }    
       digitalWrite(Pin_EN_Vanne, HIGH);
       if (millis()-Time_Vanne < Timeout_Vanne and digitalRead(Pin_Endstop_VANNE) ==1){
          ETAT_VANNE = Vanne_HCL ;}
       else{ETAT_VANNE = Vanne_ERROR;}
       if (COMMENTAIRE== true){
         Serial.print("ETAT_VANNE ");
         Serial.println(ETAT_VANNE);
       }
}

void mesure_repetabilite(){
    
      float pas = 0;

      digitalWrite(Pin_EN_Vanne, LOW);
      digitalWrite(Pin_DIR_Vanne, HIGH);
      digitalWrite(Pin_STEP_Vanne, HIGH);
      float count = 0;
      while(digitalRead(Pin_Endstop_VANNE) ==1){
            digitalWrite(Pin_STEP_Vanne, HIGH);
            delayMicroseconds(10);//min 2000  max 50
            digitalWrite(Pin_STEP_Vanne, LOW);
            delayMicroseconds(10);
          }
       digitalWrite(Pin_EN_Vanne, HIGH);
       pas = 0;
       delay(1000);

         for (int i; i<=10000;i++){
                digitalWrite(Pin_EN_Vanne, LOW);
                digitalWrite(Pin_DIR_Vanne, LOW);
                digitalWrite(Pin_STEP_Vanne, HIGH);
                count = 0;
                for (count; count<=40000;count++) { //204800*2/tour    backlash <5000  90° = 102400  360°= 409600
                  pas ++;
                  digitalWrite(Pin_STEP_Vanne, HIGH);
                  delayMicroseconds(10);//min 2000  max 50
                  digitalWrite(Pin_STEP_Vanne, LOW);
                  delayMicroseconds(10);
                }
                digitalWrite(Pin_EN_Vanne, HIGH);
                 delay(1000); 
                 
                digitalWrite(Pin_EN_Vanne, LOW);
                digitalWrite(Pin_DIR_Vanne, HIGH);
                digitalWrite(Pin_STEP_Vanne, HIGH);
                count = 0;
                while(digitalRead(Pin_Endstop_VANNE) ==1){
                      count ++;
                      pas --;
                      digitalWrite(Pin_STEP_Vanne, HIGH);
                      delayMicroseconds(10);//min 2000  max 50
                      digitalWrite(Pin_STEP_Vanne, LOW);
                      delayMicroseconds(10);
                    }
                 //Serial.print("count ");
                 //Serial.println(count);
                 Serial.print("pas ");
                 Serial.println(pas);
                 //pas = 0;
                 delay(4000);
         }
  //Serial.println(digitalRead(Pin_VANNE));
 
  }

void serialEvent(){
  while (Serial.available()){
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n'){
      stringComplete =true;
    }
  }
}

String getValue(String data, char separator, int index){
  int found =0;
  int strIndex[]={0,-1};
    int maxIndex = data.length()-1;
    for (int i = 0; i <= maxIndex && found <= index; i++){
      if (data.charAt(i) == separator || i == maxIndex){
        found++;
        strIndex[0] = strIndex[1] +1;
        strIndex[1] = (i == maxIndex) ? i +1 : i;
      }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void Envoi_Commande(byte ID,byte commande0,byte commande1,byte commande2,byte commande3,byte commande4,byte commande5,byte commande6,byte commande7){
    canMsg1.can_id  = ID; //ID
    canMsg1.can_dlc = 8;
    canMsg1.data[0] = commande0;
    canMsg1.data[1] = commande1;
    canMsg1.data[2] = commande2;
    canMsg1.data[3] = commande3;
    canMsg1.data[4] = commande4;
    canMsg1.data[5] = commande5;
    canMsg1.data[6] = commande6;
    canMsg1.data[7] = commande7;
    mcp2515.sendMessage(&canMsg1);  
}
void ASK(int numero){
      for (int i = 0; i <= 1; i++) {
        Envoi_Commande(ID,CONFIG,numero,SET_HORAIRE_t1,H1[0],M1[0]); 
        delay(50);
        Envoi_Commande(ID,CONFIG,numero,SET_HORAIRE_t2,H1[1],M1[1]); 
        delay(50);
        Envoi_Commande(ID,CONFIG,numero,SET_HORAIRE_t3,H1[2],M1[2]); 
        delay(50);
        Envoi_Commande(ID,CONFIG,numero,SET_HORAIRE_t4,H1[3],M1[3]); 
        delay(50);
        Envoi_Commande(ID,CONFIG,numero,SET_HORAIRE_t5,H1[4],M1[4]);
        delay(50);
        Envoi_Commande(ID,CONFIG,numero,SET_HORAIRE_t6,H1[5],M1[5]);
  }
}
