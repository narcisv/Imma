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
attachInterrupt(digitalPinToInterrupt(INTDOS),  Interrumpint2 , RISING );    //flanc de baixada es FALLING
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

//ProvaEnvioPerpetu(); //quan es vol enviar comandes repetitives 


//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Posar en marxa general o paro  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//if (MarxaNormal  == true)     ;

//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄      Gestio de Recepció  i execució d'ordres   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//aqui carreguem totes les variables del misstage rebut, excepte el CRC que ja s'ha controlat.switch
 if ( HeRebut == true ) { AdreClient = TramaAutoritzada[0];  // client o projecte   VILADOMAT ES EL 'A'
                                  AdrePlaca = TramaAutoritzada[1];  // destinat a es l'adreça de la placa que té aquest programa
                                  AdreRemitent =   TramaAutoritzada[2];  //
                                  Byte3 = TramaAutoritzada[3];
                                  Byte4 =TramaAutoritzada[4];
                                  Byte5 =TramaAutoritzada[5];
                                  DefinicioMissatge = TramaAutoritzada[6];
                                  Byte7 = TramaAutoritzada[7];
                                  Byte8 = TramaAutoritzada[8];
                                  NumMissatgeRebut = TramaAutoritzada[9];
                                  Byte10 =TramaAutoritzada[10];
                                  Byte11 = TramaAutoritzada[11];
   HeRebut = false;
  // ConsolaB();
   MissatgeEnExecucio = true;         // S'HA REBUT UN MISSATGE això donarà autoritzacio a certs funcionaments                       
                                   }
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄      ORDRES DE SISTEMA DE MASTER   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
                                   
 if (MissatgeEnExecucio == true  && AdreRemitent == 'A' ){  //aixó és un ordre d'un superior
                                                                          switch (DefinicioMissatge){
                                                                   case 'A':           //  POSAR EN MARXA NORMAL AMB RESET
                                                                               MarxaResetA = true ;
                                                                               ExecucioResetInicial() ;
                                                                               MissatgeEnExecucio = false ; 
                                                                               AdreRemitent = '0' ;
                                                                         break;
                                                                   case 'B':                    //  POSAR EN MARXA NORMAL CONTINUACIÓ PROGRAMES                                                             
                                                                               if ( ParoTotalZ == false )  MarxaNormalB = true ;
                                                                               MissatgeEnExecucio = false ; 
                                                                               AdreRemitent = '0' ;
                                                                         break;          
                                                                   case 'Y':                    // PARO TOTAL AMB RESERVA DE MEMORIES                                                           
                                                                               MarxaNormalB = false ;
                                                                               MissatgeEnExecucio = false ; 
                                                                               AdreRemitent = '0' ;
                                                                          break;          
                                                                    case 'Z':                 // PARO TOTAL DEFINITIU
                                                                               MarxaNormalB = false ;
                                                                               ParoTotalZ = true ; 
                                                                               MissatgeEnExecucio = false ; 
                                                                               AdreRemitent = '0' ;
                                                                          break;             
                                                                                                                                                                                                                   
                                                                                                                     }                                                                          
                                                            }

 //▄▄▄▄▄▄▄▄▄▄▄▄▄      ORDRES DE CINTA I COMANDA DESPLAÇAMENT  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄

 if (MissatgeEnExecucio == true ) {
                                                            switch (DefinicioMissatge){
                                                                   case 'T':                 // recepcio de barreres  tinc una caixa per agafar . Anuncio la meva posició x                                                                               
                                                                                 Remitent  =  AdreRemitent ;
                                                                                 PosicioDesti = Byte7  ; 
                                                                                 KlercCinta = Byte10 ;
                                                                                 BarreraRemitent  = Byte8 ;
                                                                          if (EsperantTornEmissio == false ){ EsperantTornEmissio = true ;
                                                                                 EsticOperant = true; Permisos();                                                                                                                                                                  
                                                                                 DefinicioMissatge = 1;
                                                                                 MissatgeEnExecucio = false;           }
                                                                            break;                                                                     
                                                                    case 'C':        // inici de càrrega comunicacio de dades de Master
                                                                             ComptExactitudAdreces = 0;
                                                                             if ( Remitent == 'A' ) ComptExactitudAdreces++;
                                                                             if ( PosicioDesti == Byte7 ) ComptExactitudAdreces++ ; 
                                                                             if ( BarreraRemitent == Byte8 ) ComptExactitudAdreces++ ; 
                                                                             if ( KlercCinta == Byte10 ) ComptExactitudAdreces++ ; 
                                                                             if (ComptExactitudAdreces == 4 )  { Carrega = true ; ProgramaCarrega = 10; CaixaHaPassat = 0; } 
                                                                                                                            else   { Carrega = false ; ProgramaCarrega = 0;  
                                                                                                                                                          Alarma1 = true ; Alarmes();    }
                                                                             DefinicioMissatge = 2;                                                                              
                                                                             MissatgeEnExecucio = false;                                                                            
                                                                            break;
                                                                     case'g':           // recepció dels bytes corresponent a la barrera Shuttle                                                                
                                                                                LedsHoritzontal =Byte10 ;
                                                                                LedsVertical = Byte11 ;
                                                                                DefinicioMissatge = 3;
                                                                                RebutBits = true ;
                                                                                MissatgeEnExecucio = false;  
                                                                       break;
                                                                       case 'D':                 // recepció de master . Pots descarregar caixa.                                                                              
                                                                                 Remitent  =  AdreRemitent ;  // ha de ser el master
                                                                                 PosicioDesti = Byte7  ;      //   ha de correspondre a una adreça de descarrega donada per master se
                                                                                                                          //  segons mida de la caixa.  //KlercCinta = Byte10 ; //BarreraRemitent  = Byte8 ;                                                                                                                                  
                                                                          if (EsperantTornEmissio == false ){ EsperantTornEmissio = true ;   //primer prohibim comunicacions barreres                                                                              
                                                                                 EsticOperant = true; Permisos();                                                                                                                                                                  
                                                                                 DefinicioMissatge = 4;
                                                                                 Descarrega = true ;  ProgramaDescarrega = 100;
                                                                                 MissatgeEnExecucio = false;    }
                                                                            break;
                                                                            
                                                                                             }                                                                                                                                                   
                                                            } 

//▄▄▄▄▄▄▄▄▄▄▄▄▄      ORDRES DE CARREGA CAIXA EN SHUTTLE combinat amb recepció a sobre ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄


if (Carrega == true ){
                                  Flag1 = digitalRead ( FLAG1);   Flag2 = digitalRead ( FLAG2);   //llegim contanment els flags 
                                  MicroElevacio = digitalRead ( ResetElevacio);   // llegim microrruptor d'elevació
                                  if (Flag1 == false && Flag2 == false ) CaixaHaPassat =  true;             // quan els flags 1 i 2 s'ha  tapat, considerem que esta passant un objecte

                                    switch (ProgramaCarrega){
                                               case 10:                                  //posem en marxa la potencia brush - posem en marxa motor carrega velocitat maxima  - ordenem a cinta de carregar
                                                                                              //a velocitat mitja, i passem al pas 2
                                                         if (MicroElevacio == true ) digitalWrite( PowerMotorDc,HIGH );
                                                         digitalWrite( MotorSortidaDireccio,HIGH );analogWrite( MotorSortidaPwm,255 );   // HIGH es direccio carregar caixa
                                                    if (EsperantTornEmissio == false ){ EsperantTornEmissio = true ;
                                                         OrdreACinta[6] =   'i'   ;
                                                         OrdreACinta[7] = 128;                                                        
                                                         for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = OrdreACinta[CT]; }
                                                         EnvioTrama = true ;  TX485();  
                                                         ProgramaCarrega = 20;                }
                                                                                                         
                                               break;                                               
                                               case 20:                               //quan el flag 1 queda lliure, parem el motor del shuttle i enviem missatge al motor de cina de parar i retrocedir
                                                                                         //fins a alliberar els dos flags (es la 'R')  - passem al pas seguent 3
                                                         if ( CaixaHaPassat == true && Flag1 == true ){ digitalWrite( MotorSortidaDireccio,HIGH ); analogWrite( MotorSortidaPwm,0 );}
                                                    if (EsperantTornEmissio == false ){ EsperantTornEmissio = true ;
                                                         OrdreACinta[6] =  'o' ;
                                                         OrdreACinta[7] = 64;
                                                         OrdreACinta[8] = 'R';                                                       
                                                         for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = OrdreACinta[CT]; }                                                         
                                                         EnvioTrama = true ;  TX485();
                                                         ProgramaCarrega = 30;             }
                                               break;
                                               case 30:                                //
                                                         if ( CaixaHaPassat == true && Flag1 == true && Flag2 == true ){ 
                                                                                           digitalWrite( MotorSortidaDireccio,LOW ); analogWrite( MotorSortidaPwm,25 );}  // LOW es direccio descarregar caixa
                                                         if ( Flag2 == false ) {   digitalWrite( MotorSortidaDireccio,LOW ); analogWrite( MotorSortidaPwm,0 );}   
                                                         ProgramaCarrega = 40;                         
                                               break;
                                               case 40:
                                                        if ( CaixaHaPassat == true && Flag1 == true && Flag2 == false ){  ProgramaCarrega = 50; } break;
                                                        if ( CaixaHaPassat == true && Flag1 == false ){ digitalWrite( MotorSortidaDireccio,HIGH );analogWrite( MotorSortidaPwm,25 ); }                                                                                                                        
                                                        if ( Flag1 == true && Flag2 == true ) { digitalWrite ( MotorSortidaDireccio,HIGH );analogWrite( MotorSortidaPwm,0 ); ProgramaCarrega = 3; }
                                               break;
                                               case 50:
                                                        digitalWrite( PowerMotorDc,LOW );    // parem la potencia dels motors 
                                                     if (EsperantTornEmissio == false ){ EsperantTornEmissio = true ;
                                                        FlagsComEstan[1] = BarreraRemitent ;    // enviem a la barrera corrresponent 
                                                        for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = FlagsComEstan[CT]; }
                                                        EnvioTrama = true ;  TX485();
                                                        ProgramaCarrega = 60;                  }
                                               break;
                                               case 60:                           //medició alçada de caixa, i envio ordre a Master de final de carrega                                                         
                                                    if ( EsperantTornEmissio == false && RebutBits == true ){ EsperantTornEmissio = true ;
                                                        RebutBits = false;
                                                        NombreBits = 0;
                                                        for ( int CT = 0; CT<8; CT++ ) { BitA = bitRead ( LedsVertical, CT ) ; NombreBits + BitA ; }                                                        
                                                        OrdreAMaster[6] = 'X';
                                                        OrdreAMaster[7] = NombreBits ;
                                                        OrdreAMaster[10] =LedsHoritzontal ;
                                                        OrdreAMaster[11] =LedsVertical ;
                                                         for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = OrdreAMaster[CT]; }   //
                                                        EnvioTrama = true ;  TX485();                                                                                                        
                                                        ProgramaCarrega = 70;                                                  }                                                                                           
                                               break;       
                                               case 70:       
                                                if ( EsperantTornEmissio == false  ){ EsperantTornEmissio = true ;
                                                      FinalTransport = true ;  Permisos() ;  
                                                      Carrega = false;
                                                      ProgramaCarrega = 0;
                                                      ConsolaB();                                }   break;  
                                                                            }
                              }
//▄▄▄▄▄▄▄▄▄▄▄▄▄      ORDRES DE DESCARREGAR CAIXA DE SHUTTLE A SAFATA   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄     ▄▄

if (Descarrega == true ){
                   Flag1 = digitalRead ( FLAG1);   Flag2 = digitalRead ( FLAG2);           //llegim contanment els flags 
                   MicroElevacio = digitalRead ( ResetElevacio);   // llegim microrruptor d'elevació
                                  if ( Flag1 == false || Flag2 == false ) CaixaHaPassat =  true; 
                                  if ( Flag1 == true && Flag2 == true && CaixaHaPassat ==  true ) CaixaHaSortit = true ;
               switch (ProgramaDescarrega){       
                                             case 100:  
                                                       digitalWrite( PowerMotorDc,HIGH );   // dono la potencia als motors
                                                       InclinacioElevant();  Elevant = true ; DesElevant = false ;          // fem pujar plataforma                                           
                                                       digitalWrite( MotorSortidaDireccio,LOW );  analogWrite( MotorSortidaPwm,255 );   // LOW es direccio descarregar caixa
                                                       if ( CaixaHaSortit == true )   ProgramaDescarrega = 110 ;
                                                break;
                                                case 110:                                                 
                                                  if  (  EsperantTornEmissio == false  ){ EsperantTornEmissio = true ;                                                       
                                                         OrdreAMaster[6] = 'W' ;
                                                         for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = OrdreAMaster[CT]; }   //
                                                         EnvioTrama = true ;  TX485();   ProgramaDescarrega = 120 ;  }
                                                         ComptHallDc = 0; 
                                                     break;                            
                                                 case 120: 
                                                     if  (  EsperantTornEmissio == false  ){  EsperantTornEmissio = true ; 
                                                           OrdreATots[6] = 'P';
                                                           for (int CT = 0; CT <14; CT++) {  TramaEnvio[CT] = OrdreATots[CT];  }
                                                           EnvioTrama = true ;  TX485();  }
                                                                                                                      
                                                          digitalWrite( MotorSortidaDireccio,LOW ); analogWrite( MotorSortidaPwm,0 );   // LOW es direccio descarregar caixa
                                                          InclinacioBaixant();  Elevant = false ; DesElevant = true ;   //  baixem la plataforma
                                                       if ( MicroElevacio == true ) {  digitalWrite( PowerMotorDc,LOW ); 
                                                           ComptHallDc = 0;  ProgramaDescarrega = 0 ;   }                                                        
                                                   break;
                                                               }
                                     }
//digitalWrite( PowerMotorDc,LOW );
//CaixaHaPassat =  false;
//byte VelocitatDemanadaCintaEnvioMitja = 128;   
//TopFinalEmissio
//    FinalTransport     Permisos()
                                                            
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄    Gestio d'Emissio de missatges a partir d'ordres exteriors o seguretat     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//Ordres captades per master i cosecutiva emissio
//Shuttle demana permis per circular. Es capten tots els flags, i si està en ordre es dona resposta positiva a Shuttle. Qualsevol interrupcio de flag en cami podra parar el moviment del Shuttle, que no es rea
//nudarà fins que s'hagi arreglat la interrupcio del flag, i polsat el botó de continuació.
//es capta el flag d'alçada de la caixa
//Si es prem el botó de PARO MOMENTANI, s'encén una llum vermella, i s'interrumpeix el moviment dels Shuttle. Es deixa que les cintes facin la seva funció autónoma.
//quan es torna a prèmer el PARO MOMENTANI s'encé la llum verdaa, i el funcionament es torna automàtic.
//A través de Consola de Master, o de MQTT, es pot intervenir en la velocitat i altres paràmetres de funcionament.




//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄  
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Comanda manual dels motors  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
  BotoH = digitalRead (PolsadorH );    BotoL =  digitalRead (PolsadorL );
if ( BotoH == false || BotoL == false ) { digitalWrite( PowerMotorDc,HIGH );  MicroElevacio = digitalRead ( ResetElevacio);   // llegim microrruptor d'elevació
                                                                MovimentManual();  ComptaManual = 10 ;  BaixadaFreno = true ; }
if ( Top == true ) { ComptaManual--; constrain (ComptaManual,0,100); 
                             if ( ComptaManual == 2 ) {  digitalWrite( MotorDcPlus1,LOW  ); digitalWrite( MotorDcPlus2,LOW ); 
                                                                         digitalWrite( MotorDcPwm1,LOW );digitalWrite( MotorDcPwm2,LOW ); 
                                                                         digitalWrite( PowerMotorDc,LOW ); }
                           }
if (BaixadaFreno == true ) {FrenadaMotorDc(); }

//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//♦♦♦♦♦♦♦♦♦♦♦♦♦♦   Tempos ♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦
//per engegar una tempo posar TempoA a false i en t el temps de la tempo en milisegons. Quan s'acabe, la TempoA es posa a true 

//if ( TempoA == false ){  t =10;  Tempo1();  } 

//♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦♦
 TopFinalEmissio = false;
Milis30 = millis();
if ((Milis30 - OldMilis30 ) > 20 ) { digitalWrite (Control_485_1, LOW);  TopFinalEmissio = true;  EsperantTornEmissio = false ;   }//tancament del port d'envio un temps després de l'inici de l'envio, per a deixar-li el temps de que es descarregui de manera autònoma                                                       
RX485();
 digitalWrite( PowerMotorDc,HIGH );
 digitalWrite (MotorSortidaDireccio,LOW); analogWrite (MotorSortidaPwm,255);
}//█  loop  █████████████████████████████████████████████████████████████████████████████████████████
  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   Consola     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void ConsolaB(){


Serial.print(BotoH );Serial.print(" " );Serial.print(BotoL );Serial.print(" " );Serial.print(ComptaManual );Serial.print(" " );Serial.print(MicroElevacio );Serial.print(" " );
Serial.print(ComptHallDc );Serial.print(" " );Serial.println( );
 
}//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Moviment manual    ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 

 void MovimentManual(){  if ( MicroElevacio == true ) ComptHallDc = 0;
 
if ( BotoH == false ) {  Serial.println( "2");
 digitalWrite( MotorDcPlus1,HIGH  ); analogWrite( MotorDcPwm2,128 ); 
                                     if (ComptHallDc > 100)  { digitalWrite( MotorDcPlus1,LOW  ); analogWrite( MotorDcPwm2,0 );}
                                     digitalWrite (MotorSortidaDireccio,HIGH); analogWrite (MotorSortidaPwm,128);
                                  }
if ( BotoL == false )  {  digitalWrite( MotorDcPlus2,HIGH );  analogWrite( MotorDcPwm1,128 ); 
                                     if (ComptHallDc < 30)  digitalWrite ( MotorDcPlus2,HIGH );  analogWrite( MotorDcPwm1,20 );
                                   }
ConsolaB();
} //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄ Baixada frenant  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void FrenadaMotorDc(){
if ( BotoH == true && BotoL == true && BaixadaFreno == true){
if (Top == true )  ComptFrenada++;
// digitalWrite (MotorSortidaDireccio,HIGH); analogWrite (MotorSortidaPwm,125);
if ( ComptFrenada >100 ) ComptFrenada = 0;
if (ComptFrenada< 20)  {  digitalWrite( MotorDcPlus2,HIGH  ); analogWrite( MotorDcPwm1,50 ); } 
if (ComptFrenada >20)                                {   
                                     digitalWrite ( MotorDcPlus1,LOW  );  digitalWrite ( MotorDcPlus2,LOW );
                                     analogWrite( MotorDcPwm1,255 );  analogWrite( MotorDcPwm2,225 );       }
                                     
                                      if ( MicroElevacio == true )  {BaixadaFreno = false;  digitalWrite ( MotorDcPlus1,LOW  );  digitalWrite ( MotorDcPlus2,LOW );
                                                                                      digitalWrite( MotorDcPwm1,LOW );  digitalWrite( MotorDcPwm2,LOW );    
                                                                                      digitalWrite( PowerMotorDc,LOW ); ComptFrenada = 0;
                                                                                    }                                                  
}


}//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Moviment Automatic Elevant ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void InclinacioElevant(){
   digitalWrite( MotorDcPlus1,HIGH  ); analogWrite( MotorDcPwm2,128 );
    if  ( ComptHallDc >= 90 ) digitalWrite( MotorDcPlus1,LOW  ); analogWrite( MotorDcPwm2,0 );


} //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Moviment Automatic Baixant ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void InclinacioBaixant(){
    digitalWrite( MotorDcPlus2,HIGH );  analogWrite( MotorDcPwm1,64 );
    if ( MicroElevacio == true ) digitalWrite( MotorDcPlus2,LOW );  analogWrite( MotorDcPwm1,0 );
  

 } //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   interrupcions     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄     ▄▄▄

//void Interrumpint1(){ ;  }//b++; Flash4 = false;  Flash1 = false;   Micros0= micros(); digitalWrite (Out3__,LOW) ; digitalWrite (Out3Pos,LOW) ;
void Interrumpint2(){ if (Elevant == true )  ComptHallDc++; if (ComptHallDc > 100)  { digitalWrite( MotorDcPlus1,LOW  ); analogWrite( MotorDcPwm2,0 ); }
                                 if (DesElevant == true )  ComptHallDc--; if (ComptHallDc < 30)  digitalWrite( MotorDcPlus2,HIGH );  analogWrite( MotorDcPwm1,20 ); }
                                 
                                  
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄   ALARMES     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void Alarmes(){
 if (Alarma1 == true )  { AlarmesCom[7] = 1 ;    //No coincideix algun element d'adreça una vegada arribat a destinació. Comprovació creuada
                                  for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = AlarmesCom[CT]; }
                                  EnvioTrama = true ;  TX485(); 
                                  }
 
} //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄   PERMIS DE COMUNICACIO A BARRERES DE LES CINTES     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void Permisos(){
  EsperantTornEmissio = true ;

if (EsticOperant == true ) {   OrdreATots[6] = 'S';          //fem silenci barreres
                                               for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = OrdreATots[CT]; }
                                               EnvioTrama = true ;
                                               TX485();
                                               EsticOperant =false;
                                            }
if (FinalTransport == true ) {   OrdreATots[6] = 'P';   //deixem parlar barreres
                                                OrdreATots[7] = Prioritari ;                                   
                                               for (int CT = 0; CT <14; CT++) {TramaEnvio[CT] = OrdreATots[CT]; }
                                               EnvioTrama = true ;
                                               TX485();
                                               FinalTransport =false ;
                                             }
} //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄  Reset Inicial    ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄

 void ExecucioResetInicial(){
  // digitalWrite(LedA,HIGH);
//ParoTotalZ

} //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄  EMISSIO  TX485    ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// aquest subprograma envia una trama "  TramaEnvio[]   " de 14 bytes. en els dos últimsTramaEnvio[12]  i TramaEnvio[13] la trama col.loca el CRC, i la envia tota
//els codis d'aquesta trama estan establerts en l'Excel  : D:\Imaginem\IMA_TECNIC\Projectes\__IMAGINEM PROJECTES\IMA ANALOGIC CONTROL\LLENGUATGE 485 GENERACIO 2019\LLENGUATGE485
//les configuracions inicials es troben en el fitxer adjunt explicacions
//s'ha de tenir compte per quin canal serie s'envien les trames, i canviar-lo si cal  " Serial.write (TramaEnvio[CT4]) ; Serial1.write (TramaEnvio[CT4]) ; Serial2.write (TramaEnvio[CT4]) ;  Serial3.write (TramaEnvio[CT4]) ; "

void TX485() {
//___012____CALCUL CRC I CARREGA EN LA TRAMA D'ENVIO_________________________________________________________________________________                                                                   
NumMissatgeEnviar++;
           if (EnvioTrama == true )  {       //Serial.println ("3");                                                                                                                                //calculs del CRC
                             crc = 0xFFFF ;
                             for ( int CT = 0 ; CT < 12; CT++ ){ //ElementsTrama
                             crc = crc   ^=   TramaEnvio[CT];      
                             for (int CT11 = 8 ; CT11  != 0 ; CT11--) 
                                    { if ((crc & 0x0001) !=0 ) {    crc = crc >> 1;   crc = crc ^= 0xA001;   }  else { crc = crc >> 1;} }    }                          
                                    crcBaixCalculat = crc & 0xFFFF;       crcAltCalculat =  crc >>8;   
                          TramaEnvio[12] = crcBaixCalculat; TramaEnvio[13] = crcAltCalculat;                                                               //una vegada calculats els CRC els posem en a trama que sera enviada

//___014  envio COMUNICACIO   ___forma part del 012______________________________________________________________________________
                    //digitalWrite(Led, HIGH); delay (500); digitalWrite(Led, LOW); 
                    digitalWrite (Control_485_1, HIGH); OldMilis30 = millis();  delay (1);                       //obre la sortida 485 a TX, que es tancara després de 35 ms
                                           Serial1.write (0x3A) ; delay(1);                                                                                                      //posem primer el byte BB per a informar de principi de trama
                                           for (int CT4 = 0; CT4 <14 ; CT4++)   { Serial1.write (TramaEnvio[CT4]) ;  TramaChar[CT4] = TramaEnvio[CT4];  }  //envio de trama i posta a 0 de la trama per a evitar errors        TramaEnvio[CT4] = 0                                    
                                            EnvioTrama = false;                                                                                                                    //tanquem l'envio de trama                                                                                                                     
                                                     }   
}  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ RECEPCIO RX 485 ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//en aquest subprograma es reb una trama per RX TX o 485, documentada amb CRC. una vegada autoritzada i controlada la autenticitat de l'envio gràcies al CRC, la trama sortint té
//la referència "  TramaAutoritzada[CT]  ", si es vol treballar sota forma de bytes, decimal, binari, o bé té la forma "  TramaChar[CT]  " si es vol utilitzar sota forma de caràcters.
//unsigned long NovaCoordinada = 0;
//byte NivellActual = 0;    //nivell al qual estem
//byte NivellDesitjat = 0;


void RX485(){  
//||||||||||||||||||||||||||||||||||||||||||||||||||||  RECEPCIO  PER CONSOLA||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||5||||||||||||||||||||||||||||||||||||||||||
if ( Serial.available()  )   { Consola = Serial.readString();  NivellGravatManual  = Consola.toInt() ;
             if ( Consola == "?") { OldFase = Fase;  Fase = 1;  Consola.remove(0);  MostraPantalla =1;  }
             if ( Consola == "CONSOLA" ) {  OldFase = Fase;  Fase = 2; Consola.remove(0);  MostraPantalla =1; }    // fase de consola base
             if (Consola ==  "GRAVAR" )   {  OldFase = Fase; Fase = 3;   Consola.remove(0);  MostraPantalla =1; }    // FASE DE GRAVAR COORDINADES
             if ( Consola == "SORTIR" )     {  OldFase = Fase;  Fase = 9; Consola.remove(0); MostraPantalla =1; }   // fase per sortir de consola
             if ( Consola == "MARXA ON" )      { OldFase = Fase;  Fase = 8; Consola.remove(0); MostraPantalla =1; }   // fase de posar en marxa sistema  ENVIO MISSATGE RUN TOTAL I SORTIR DE CONSOLA
             if ( Consola == "PARO OFF" )      { OldFase = Fase;  Fase = 7; Consola.remove(0); MostraPantalla =1; }    //fase de parar sistema envio Missatge de PARO TOTAL        
              if ( Consola == "#" )   {  Consola.remove(0);  Fase = 1;  }    // PER SORTIR A CONSOLA
             
        //     if ( Consola == "NOU NIVELL" )   {  Consola.remove(0);  NouNivell();  }    //anem a gravacio de coordinades              

          //  if ( EntradaVariable == true && NivellGravatManual > 0 )   {  Serial.print (Consola) ;  Consola.remove(0) ;  GravaCoordinades() ; }  //NivellGravatManual  = Consola.toInt() ;  Serial.print ('  |  ') ;
                                       }                                              
                   
 //||||||||||||||||||||||||||||||||||||||||||||||||||||  RECEPCIO  ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||5||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
 //______ Recepcio d'una trama de 14 bytes, incloent 2bytes de CRC._precedentment rebem un byte d'identificacio 0x3A_________________________________
if (Serial1.available()>14  )  { //Serial.println ( '2') ;
                                           ByteControl = Serial1.read();                                                                                      
                                           if (ByteControl == 0x3A) { ComRecepcions++; Llegit = 1;  
                                           for ( int CT6 = 0; CT6 <14;  CT6++) { TramaRecepcio[CT6] = Serial1.read(); TramaChar1[CT6] = TramaRecepcio[CT6];  }          
                                                                                 }                                       
                                             CrcCalculOk = false ;
                                           if (  TramaRecepcio[1] == AdrePlaca || TramaRecepcio[1] == AdreTots) 
                                           {  ComAcceptada = true;   }                                      
                                           }                   
            if (ComAcceptada == true)  
            {         //  |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
                     for   (int CT5 = 0; CT5 < 14 ; CT5++)    {   TramaCalculCrc[CT5] = TramaRecepcio[CT5];   TramaRecepcio[CT5]= 0;   }            
                     crcBaixLlegit = TramaCalculCrc[12]; 
                     crcAltLlegit = TramaCalculCrc[13];             
//______________________________________________________________________________________________________________________________     
//______Calcul CRC _______________________________________________________________________________________________________________
                                 crc = 0xFFFF ;
                                 for (int CT = 0 ; CT < 12; CT++ ){ //ElementsTrama
                                 crc = crc   ^=   TramaCalculCrc [CT];      
                                 for (ComptCalculOr = 8 ; ComptCalculOr  != 0 ; ComptCalculOr--) 
                                            { if ((crc & 0x0001) !=0 ) {    crc = crc >> 1;   crc = crc ^= 0xA001;   }  else { crc = crc >> 1;} }    }                          
                                 crcBaixCalculat = crc & 0xFFFF;       crcAltCalculat =  crc >>8;       
                            //     Serial.print( crcBaixLlegit);  Serial.print( "  ");  Serial.print( crcAltLlegit);  Serial.print( "  ");  Serial.print( crcBaixCalculat);  Serial.print( "  ");  Serial.print( crcAltCalculat);  Serial.println( "  ");            
//_____________________________________________________________________________________________________________________________                   
//______autorització de presa en compte de trama_____________________________________________________________________

                                if ( crcAltCalculat == crcAltLlegit &&  crcBaixLlegit== crcBaixCalculat)       //la trama passa a ser acceptada quan els CRC corresponen
                                       {   CrcCalculOk = true;   HeRebut = true;       //   comptadorOKs++;                 
                                           for (int CT = 0; CT <14; CT ++) { TramaAutoritzada[CT] = TramaCalculCrc[CT];  TramaChar[CT] = TramaCalculCrc[CT]; }
                                        //   for (int CT = 0; CT <6; CT ++) {      //envio trama petita de aceptació
 // Serial.print( "M     :   ") ; Serial.print ( crcAltCalculat) ;Serial.print ( " ") ;Serial.print ( crcAltLlegit ) ;Serial.print ( " ") ;Serial.print (  crcBaixCalculat ) ; Serial.print ( " ") ;Serial.println ( crcBaixLlegit ) ; 
                                                                                            }                                                                                                                              
            }  else  {  HeRebut = false;    for (int CT = 0; CT <14; CT++) { TramaCalculCrc[CT] = 0;TramaAutoritzada[CT] = 0; TramaRecepcio[CT] = 0; }                                                                                                                           
                          }
}//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 

//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   ENVIO ACUS REBUT      ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//S'envia un missatge de recepció de ordre quan no es precisa una resposta.
   
void EnvioAcusRebut(){
                    if (AcusRebut == true ){
                                                        TramaOK[1] = AdreEnviador; 
                                                        TramaOK[2] = AdrePlaca;
                                                        TramaOK[5] = NumMissatgeRebut;
                                                        TramaOK[9] = NumMissatgeEnviar;                                                        
                                                        for ( int CT = 0 ; CT < 12; CT++ ) TramaEnvio[CT] = TramaOK[CT];
                                                        EnvioTrama = true;
                                                        TX485();
                                                        AcusRebut =  false ;
                                                        Serial.println ("contestacio: "  );
                                                        for (int CT = 0; CT< 14 ; CT++ ){ Serial.print(TramaOK[CT]);Serial.print("  "); } Serial.print(" ▄  ");
                                                       }                            
 }//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄

  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   Gestio d'Envio  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//només en el cas de prova 
void ProvaEnvioPerpetu(){
if (TopSegon == true ) {ComptSegons++;  }
if (ComptSegons ==4) {ComptSegons = 0;
                           TramaFLAGpregACarro[9] = NumMissatgeEnviar;
                          for ( int CT = 0 ; CT < 12; CT++ ) TramaEnvio[CT] = TramaFLAGpregACarro[CT];   //demanemal carro que ens envii esta de barrera
                          EnvioTrama = true;
                          TX485();                                               
}}  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 

//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   temporitzacio     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
void Tempo1(){   if ( A == false ) { OldMilisTempo = millis();    t1 = t ;  A = true;}
                         MilisTempo = millis(); 
                         if ( OldMilisTempo + t1 < MilisTempo ) TempoA = true; A = false;                                   
                      
}   //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄


   /*   
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   Gestio Trama Rebuda   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄


if (HeRebut == true ) {  
                                                 Coincidencia = 0;  for (int CT = 0; CT< 7 ; CT++ ){ 
                                                 if( TramaAutoritzada[CT] == TramaFLAGrespDeMob[CT]) {    Coincidencia++; }
                                                                                                        }                                                                                                          
                                                 if ( Coincidencia == 7 ) { PreguntaFlag = true ;
                                                                                      EnviarEstat = true ;
                                                                                      AdreEnviador = AdrePlaca ;
                                                                                      NumMissatgeRebut = TramaAutoritzada[9];                                                                                     
                                                                                      FlagByte1 = TramaAutoritzada[7] ;
                                                                                      FlagByte2 = TramaAutoritzada[8] ;
                                                                                      Flag1 = bitRead ( FlagByte1,0) ;
                                                                                      Flag2 = bitRead ( FlagByte1,1) ;
                                                                                      EntradaMultiplexatLeds1 = TramaAutoritzada[10] ;
                                                                                      EntradaMultiplexatLeds2 = TramaAutoritzada[11] ;                                                                                      
                                                                                      HeRebut = false;
                                                                                      AcusRebut = true ;
                                                                                      //EnvioAcusRebut();
                                                                                      Serial.println ("RECEPCIO: "  );
                                                                               
                                                                                      for (int CT = 0; CT< 7 ; CT++ ){ (TramaChar[CT])= (TramaAutoritzada[CT]); Serial.print(TramaChar[CT]); Serial.print("  "); }
                                                                         
                                                                                      Serial.print("  ");
                                                                                      for (int CT = 7; CT< 14 ; CT++ ){ Serial.print(TramaAutoritzada[CT]);Serial.print("  "); } Serial.println(" █  ");
                                                                                                                                                                            
                                                                                      Serial.print(TramaAutoritzada[10],BIN);Serial.print("  "); Serial.println(TramaAutoritzada[11],BIN);
                                                                                      
                                                                                      Serial.print(Flag1); Serial.print("  ");Serial.print(Flag2);Serial.print("  ");
                                                                                      Serial.print(EntradaMultiplexatLeds1,BIN); Serial.print("  "); Serial.print(EntradaMultiplexatLeds2,BIN); Serial.println("  ");
                                                                                      
                                                                                      }

}







                                        
                                                                                                                    

             TramaFLAGresposta[7]   = FlagByte1;
             TramaFLAGresposta[8]   = FlagByte2;                                     
             TramaFLAGresposta[9]   = NumMissatgeRebut;
             TramaFLAGresposta[10] = EntradaMultiplexatLeds1;
             TramaFLAGresposta[11] = EntradaMultiplexatLeds2;

             for ( int CT = 0 ; CT < 12; CT++ ) { TramaEnvio[CT] = TramaFLAGresposta[7] ; }     
//             TX485();                                  
EnviarEstat = false ; PreguntaFlag = false ;  HeRebut == true;




 //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄            
*/ 
