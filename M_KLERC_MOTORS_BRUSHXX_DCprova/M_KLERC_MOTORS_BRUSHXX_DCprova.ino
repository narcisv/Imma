// PILOTATGE MOTORS BRUSH PER PLACA KLERC MES DRIVER XINES
//ULTIM PROGRAMA COMENÇAT EL 22/09/19
// PROGRAMA BASE DE COMUNICACIÓ ENTRADA I SORTIDA, I SELECCIO D'ORDRES
// UTILITZACIÓ DE CONSOLA PER A DONAR ORDRES EXTERNS, O PER REBRE INFORMACIÓ DIRECTE
// A BASE DE M_KLERC
// UTILITZACIÓ DEL PROGRAMA MODIFICAT PER A PILOTAT ELS MOTORS DEL SHUTTLE 


#define Control_485_1 6          //0=recepcio RX  1= transmissio TX   Control de flux RS485 ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
#define MotorSortidaPwm 9             // Commutador de direccio del motor
#define PowerMotorDc  26   
#define MotorSortidaDireccio 11            // Commutador de direccio del motor
#define MotorDcPlus1 51         // Comandes del motor Elevació 
#define MotorDcPwm1 12       // Comandes del motor Elevació per PWM
#define MotorDcPlus2 52         // Comandes del motor Elevació 
#define MotorDcPwm2 8          // Comandes del motor Elevació per PWM
#define Hall1MotorBrush 39      //entrada Hall del motor brush per controlar la seva velocitat
#define HallMotorDc 38            //entrada Hall del motor dc
#define Hall1BrushInter 21        //interrrupcio del hall brush  motor 1
#define HallMotDcInter 2         // interrupcio del hall motor dc
#define OnOffMotors  40        // on/off motors  ATENCIO normalment entrada a 1, que s'ha de passar a '0' perquè hi ha un inversor en 40 normalment a 0, passara a 1 quan entrada 3 del circuit passa a 0
#define  ResetElevacio  61      //entrada de seguretat s'activa a 1 quan l'elevadora arriba a baix.
#define  SobrepassamentElevacio 59    //entrada de seguretat, s'activa a 1 quan plataforma eleva massa.
#define  LedIlluminacio 10               // sortida per encendre leds d'il.luminació
#define  TempMotorBrush A0   //temperatura motor   a 0º sortida 400mV, +19,5 mV/ºC
#define  TempMotorDc  A1     //Temperatura motor DC    a 0º sortida 400mV, +19,5 mV/ºC
#define  PolsadorH 27             //polsadors de funcionament manual
#define PolsadorL 25
#define LedA 28                     //led indicador
#define FLAG1 56        // A2 passat a digital
#define FLAG2 59        // A5 passat a digital
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   VARIABLES SUBJECTES A CANVI PER PLACA     ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
byte AdrePlaca = 'C';  // S DE PLACA MOTORS BRUSH + DC    ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
byte AdreTots = 'T';   //  T de Tots                                                    ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
byte Prioritari = 'm' ; // m prioritat cinta magatzem,  v cinta botiga  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒    Gestio del Temps     ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
unsigned long CincCentsMicros;     // Variable de Temps Actual   per fer el rellotge       
unsigned long SisCentsMicros;   // Variable de Temps Anterior per fer el rellotge
unsigned long DosCentMicros;
unsigned long MicroSegons;
unsigned long UnSegon;
unsigned long  CentMilis;
bool TopSegon;
unsigned long Segons;
bool PostaAZero= false ;
bool Top = true ;  
bool Top1 = true ;
bool Top2 = true ;
bool ControlTop1;
bool ControlTop2;
int ComptSegons;

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   EMISSIO TX 485     ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
int NumMissatgeEnviar = 0;
bool EnvioTrama = false ;  //EnvioTrama = 1 funcionament en TX
unsigned int crc = 0xFFFF ;
byte TramaEnvio[14] ;
char TramaChar[14] ;
byte crcBaixCalculat = 0x00;
byte crcAltCalculat  = 0x00;
byte crcBaixLlegit = 0x00;
byte crcAltLlegit  = 0x00;
//pinMode(Control_485_1,OUTPUT);        // Sortida del control de flux del RS485    
unsigned long Milis30 = 0;
unsigned long OldMilis30 = 0;

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  RECEPCIO RX 485  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
String Consola ;
int NivellGravatManual = 0;
int OldFase = 0 ;
int Fase = 0;      //conmutador de les diferents fases del funcioname
bool MostraPantalla = 0;     
byte ByteControl = 0;
int ComRecepcions = 0; //comptador  
bool Llegit = false;  // indica que hem llegit una trama amb succes
byte TramaRecepcio[14];
char TramaChar1[14] ;
byte TramaAutoritzada[14] ;
byte TramaCalculCrc[14];
bool CrcCalculOk = false;
bool ComAcceptada = false;
int ComptCalculOr = 0 ;
bool HeRebut = false ;  //indicador de recepció de trama correcta a explotar

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   ENVIO ACUS REBUT   ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
bool AcusRebut = false;
byte AdreEnviador;

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   STRINGS COMUNICACIO   ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
byte TramaFLAGpregACarro[14] = {'A','B','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA CARRO  atenció s'ha de canviar el receptor (adreça de placa ) segona lletra per cada programa
byte TramaFLAGpregACintMag[14] = {'A','m','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;  //DE MASTER A BARRERA DE CINTA MAGATZEM
byte TramaFLAGpregACintVen[14] = {'A','V','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA DE CINTA VENEDORS
byte TramaFLAGpregATobMag[14] = {'A','Z','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA TOBOGAN MAGATZEM
byte TramaFLAGpregATobBotiga[14] = {'A','X','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;    //DE MASTER A BARRERA TOBOGAN BOTIGA
byte TramaFLAGpregATots[14] = {'A','T','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA TOTS 

byte TramaFLAGrespDeMob[14] = {'A','A','B','F','L','A','G',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE MOBIL  BARRERA  A MASTER
byte TramaFLAGrespDeCinMag[14] = {'A','A','m','F','L','A','G',0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //  DE CINTA MAGATZEM BARRERA  A MASTER
byte TramaFLAGrespDeCinVen[14] = {'A','A','V','F','L','A','G',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE CINTA VENEDORS BARRERA A MASTER 
byte TramaFLAGrespTobMag[14] = {'A','A','Z','F','L','A','G',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE TOBOGAN MAGATZEM  BARRERA  A MASTER 
byte TramaFLAGrespTobBotiga[14] = {'A','A','X','F','L','A','G',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE TOBOGAN BOTIGA  BARRERA  A MASTER 

byte TramaOK[14] = {'A','T','A','O','K',0x00,'<',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE TOBOGAN BOTIGA  BARRERA  A MASTER 


byte FlagsComEstan[14]= {'A','?','C','F','L','A','G',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ; 
byte AlarmesCom[14]= {'A','T','?','A','L','A','R',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ; 
byte OrdreATots[14]= {'A','T','C','O','R','D','S',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;  //fer silenci cintes si posem P es permet parlar cintes
byte OrdreAMaster[14]= {'A','A','C','O','R','D','V',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;
byte OrdreACinta[14]= {'A','?','C','C','A','R','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   GESTIO DE RECEPCIÓ   ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 
byte  AdreClient = 0;
byte  AdreRemitent = 0;
byte  Remitent = 0;
byte  BarreraRemitent = 0 ;
byte  Byte3 = 0;
byte  Byte4 = 0;
byte  Byte5 = 0;
byte  DefinicioMissatge = 'A';
byte  Byte7 = 0;
byte  Byte8 = 0;
byte  NumMissatgeRebut = 0;
byte  Byte10 = 0;
byte  Byte11 = 0;
bool MissatgeEnExecucio = 0;
bool TopFinalEmissio = false;
 //▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   TEMPORITZACIONS  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
unsigned long t = 0;
unsigned long t1 = 0;
bool TempoA = true;
bool A = false;
unsigned long OldMilisTempo;
unsigned long MilisTempo;
unsigned long Micros10;
unsigned long Micros20;
unsigned long Micros0;
unsigned long Micros11 = 0;
unsigned long Micros21 = 0;

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   COMPTATGE VOLTES DELS MOTORS  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
int ComptBrush = 0;
int ComptMotorDC = 0;
int b=0;
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  FUNCIONAMENT MANUAL▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 
bool BotoH = false;
bool BotoL = false;
int ComptaManual = 0;
bool MicroElevacio = 0;
bool BaixadaFreno = 0 ;
int ComptFrenada = 0;
int ComptManual2 = 0;
bool ParoManual = false; 
bool ArribatAZero = false;
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  MOVIMENTS DE TRANSLACIÓ I ORDRES GENERALS▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
bool MarxaNormalB = false  ;
bool ParoTotalZ = true ;
bool ParoPuntualY = false ;
bool MarxaResetA = true ;
bool Canal485Tancat ;
bool Canal485Obert ;
bool EsticOperant = false ;    // es posa a 1 quan hem de silenciar les cintes
bool FinalTransport = false ;  // es posa a 1 quan hem de deixar parlar les cintes
bool EsperantTornEmissio = false;
byte PosicioDesti = 0;
int ComptExactitudAdreces = 0;
byte KlercCinta = 0 ;
bool Alarma1 = 0; 
bool CaixaHaPassat = 0;
bool Flag1 = 0;
bool Flag2 = 0;
byte LedsHoritzontal  = 0;
byte LedsVertical  = 0;
bool BitA;   // pel comptatge dels bits de l'alçada de la caixa
int NombreBits = 0 ;
bool RebutBits = false ; 
bool CaixaHaSortit = false ;

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  MOVIMENT AUTOMATIC ROTACIO CINTA  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
byte VelocitatDemanadaCintaEnvioAlta = 255;
byte VelocitatDemanadaCintaEnvioMitja = 128;
byte VelocitatDemanadaCintaEnvioBaixa = 30;
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 
 //▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   MOVIMENT AUTOMATIC INCLINACIÓ  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
unsigned long ComptHallDc = 0;
bool Elevant = false;
bool DesElevant = false;
 //▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 
 //▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   MOVIMENT CARREGA I DESCARREGA  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
bool Carrega = false ;
bool Descarrega = false ;
bool MotorCarrega = false ;
bool MotorDescarrega = false ;
int ProgramaCarrega = 0 ;
int ProgramaDescarrega = 0;


 //▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒   INTERRUPCIONS    ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
const int INTZERO = 21;
const int INTDOS = 2;
const int INTTRES = 20;
bool ParoCicle2 = true;
bool ParoCicle3 = true;
bool ParoCicle4 = true;
bool ParoCicle5 = true;
bool ParoCicle6 = true;

int ComptadorPassos1 = 0;
int ComptadorPassos2 = 0;
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒




#include <avr/wdt.h> 
#define RESETWATCHDOG
#include <EEPROM.h> 
#include <SoftwareSerial.h>

void setup(){          //________________________________ Void Setup ________________________________
pinMode(Control_485_1,OUTPUT); 
pinMode(MotorSortidaPwm,OUTPUT); 
pinMode(PowerMotorDc,OUTPUT); 
pinMode(MotorSortidaDireccio,OUTPUT); 
pinMode(MotorDcPlus1,OUTPUT); 
pinMode(MotorDcPwm1,OUTPUT); 
pinMode(MotorDcPlus2,OUTPUT); 
pinMode(MotorDcPwm2,OUTPUT); 
pinMode( LedIlluminacio ,OUTPUT); 
pinMode( LedA ,OUTPUT); 


pinMode( Hall1BrushInter ,INPUT); 
pinMode( Hall1MotorBrush ,INPUT);
pinMode( HallMotorDc ,INPUT);
pinMode( HallMotDcInter ,INPUT);
pinMode( OnOffMotors ,INPUT);
 
 pinMode( ResetElevacio ,INPUT);       
 pinMode( SobrepassamentElevacio ,INPUT);
pinMode( TempMotorBrush ,INPUT);
pinMode( TempMotorDc ,INPUT);
pinMode( PolsadorH ,INPUT_PULLUP);
pinMode( PolsadorL ,INPUT_PULLUP);
pinMode( INTZERO ,INPUT);
pinMode( INTDOS ,INPUT);
pinMode( FLAG1 ,INPUT);
pinMode( FLAG2 ,INPUT);
Serial.begin(9600);     
Serial1.begin(9600);  // Obrir el port de comunicacions a 9600 baudis
Serial.setTimeout(40); 
Serial1.setTimeout(40);

//attachInterrupt(digitalPinToInterrupt(INTZERO), Interrumpint1 , RISING );   //flanc de pujada es RISING
//attachInterrupt(digitalPinToInterrupt(INTDOS),  Interrumpint2 , RISING );    //flanc de baixada es FALLING  ███
//attachInterrupt(digitalPinToInterrupt(INTTRES), Interrumpint3, CHANGE );  //qualsevol flanc es CHANGE

 wdt_disable();
 wdt_enable(WDTO_8S); 
delay(2000);
}


void loop() {  //█  loop  ███████████████████████████████████████████████████████████
 wdt_reset();
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄      Gestio del Temps     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄

Top = false;
Top1 = false;
Top2 = false;
TopSegon = false;
MicroSegons=micros();

 if ( CentMilis + 100000 <= MicroSegons ) {  Top = true; ControlTop1 = true ;  ControlTop2 = true ; CentMilis = micros();  } //durada en us del cicle total d'un led
//  if ( CincCentsMicros +300 <= MicroSegons && ControlTop1 == true ) {  Top1 = true;    ControlTop1 = false ;  }  // punt de medició . s'ha de tenir en compte que lect analogica dura 160us.
//  if ( CincCentsMicros + 850 <= MicroSegons && ControlTop2 == true) {  Top2 = true;  ControlTop2 = false ;   }// moment de parada per no generar un consum constant llarg per les alimentacions
 if (UnSegon + 1000000 <= MicroSegons ) { TopSegon = true; Segons++; UnSegon = MicroSegons ; PostaAZero = false ; }
                           if ( MicroSegons < 4000 && PostaAZero == false ) 
                           { CentMilis = 0 ;   UnSegon = 0; 
                            OldMilisTempo = 0; MilisTempo = 0 ; OldMilis30 = 0; PostaAZero = true ;}  //posta a '0' per overflow            
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//▄▄▄▄▄▄▄  
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Comanda manual dels motors  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
  
  BotoH = digitalRead (PolsadorH );   // BotoL =  digitalRead (PolsadorL );
  if ( BotoH == false ) { digitalWrite( PowerMotorDc,HIGH );     // llegim microrruptor d'elevació
                                                                MovimentManual();  ComptaManual = 10 ;  BaixadaFreno = true ; }
 // if ( Top == true ) { ComptaManual--; constrain (ComptaManual,0,100); 
//                             if ( ComptaManual == 2 ) {  digitalWrite( MotorDcPlus1,LOW  ); digitalWrite( MotorDcPlus2,LOW ); 
 //                                                                        digitalWrite( MotorDcPwm1,LOW );digitalWrite( MotorDcPwm2,LOW ); 
 //                                                                        digitalWrite( PowerMotorDc,LOW ); }
  //                           }
                           
  //if (  BaixadaFreno == true  &&  BotoH == true  ) {   FrenadaMotorDc();  }███████
   MicroElevacio = digitalRead ( ResetElevacio); 
   if ( MicroElevacio == true ) { ComptFrenada = 0; ParoManual = false; ComptManual2= 0; }
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//♦♦♦♦♦♦♦♦♦♦♦♦♦♦   Tempos ♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦
//per engegar una tempo posar TempoA a false i en t el temps de la tempo en milisegons. Quan s'acabe, la TempoA es posa a true 

//if ( TempoA == false ){  t =10;  Tempo1();  } 

//♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦
 TopFinalEmissio = false;
Milis30 = millis();
if ((Milis30 - OldMilis30 ) > 20 ) { digitalWrite (Control_485_1, LOW);  TopFinalEmissio = true;  EsperantTornEmissio = false ;   }//tancament del port d'envio un temps després de l'inici de l'envio, per a deixar-li el temps de que es descarregui de manera autònoma                                                       
//RX485();
// digitalWrite( PowerMotorDc,HIGH );
// digitalWrite (MotorSortidaDireccio,LOW); analogWrite (MotorSortidaPwm,255);
}//█  loop  █████████████████████████████████████████████████████████████████████████████████████████
  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   Consola     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void ConsolaB(){


Serial.print(BotoH );Serial.print(" " );Serial.print(ComptFrenada );Serial.print(" " );Serial.print(ComptManual2 );
Serial.print(" " );Serial.print(MicroElevacio );Serial.print(" " );
Serial.print(ParoManual );Serial.print(" " );Serial.println( );
 
}//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Moviment manual    ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 

 void MovimentManual(){  if ( MicroElevacio == true ) ComptHallDc = 0;
 
if ( BotoH == false && ParoManual == false ) {
                                                                      digitalWrite( MotorDcPlus1,HIGH  );  digitalWrite ( MotorDcPlus2,LOW   ); digitalWrite (MotorDcPwm1,LOW );
                                                            if (ComptManual2 <= 2 ) {    analogWrite( MotorDcPwm2,200 ); Serial.println("  aaa " );  }
                                                            if (ComptManual2 > 2 )   {   analogWrite( MotorDcPwm2,128 ); ArribatAZero = true ;  Serial.println("  bbb " );  }

                                                          if ( Top == true )  { ComptManual2++; constrain (ComptManual2 , 0, 50); }
                                                          if (ComptManual2 == 27) { ParoManual = true; Serial.println("  CCC " ); 
                                                                         digitalWrite ( MotorDcPlus1,LOW  );  digitalWrite ( MotorDcPlus2,LOW   );
                                                                         digitalWrite( MotorDcPwm1,HIGH );  analogWrite ( MotorDcPwm2,200 ); 
                                                                                                     } 
                                                                         }

                                    // if (ComptHallDc > 100)  { digitalWrite( MotorDcPlus1,LOW  ); analogWrite( MotorDcPwm2,0 );}
                               //      digitalWrite (MotorSortidaDireccio,HIGH); analogWrite (MotorSortidaPwm,128);   // motor rotacio cinta
                                  
//if ( BotoL == false )  {  digitalWrite( MotorDcPlus2,HIGH );  analogWrite( MotorDcPwm1,128 ); 
//                                     if (ComptHallDc < 30)  digitalWrite ( MotorDcPlus2,HIGH );  analogWrite( MotorDcPwm1,20 );
//                                   }

ConsolaB();
} //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄

/*
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄ Baixada frenant  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void FrenadaMotorDc(){
if ( BotoH == true  && BaixadaFreno == true){ 
if (Top == true )  ComptFrenada++;
// digitalWrite (MotorSortidaDireccio,HIGH); analogWrite (MotorSortidaPwm,125);
if ( ComptFrenada >100 ) ComptFrenada = 0;
if (ComptFrenada< 17)  {  digitalWrite( MotorDcPlus2,HIGH  ); analogWrite( MotorDcPwm1,50 ); } 
if (ComptFrenada >17)                                {   
                                     digitalWrite ( MotorDcPlus1,LOW  );  digitalWrite ( MotorDcPlus2,LOW );
                                     analogWrite( MotorDcPwm1,255 );  analogWrite( MotorDcPwm2,225 );       }
                                     
       if ( MicroElevacio == true  && ArribatAZero == true )    {ArribatAZero = false;  BaixadaFreno = false;  digitalWrite ( MotorDcPlus1,LOW  );  digitalWrite ( MotorDcPlus2,LOW );
                                                        digitalWrite( MotorDcPwm1,HIGH );  digitalWrite( MotorDcPwm2,HIGH );    
                                                        digitalWrite( PowerMotorDc,LOW ); ComptFrenada = 0; ParoManual = false; 
                                                        ComptManual2 = 0;Serial.println("  bbb " );
                                                                                    }                                                  
                                                                           }


}//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄

 */


 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄            
