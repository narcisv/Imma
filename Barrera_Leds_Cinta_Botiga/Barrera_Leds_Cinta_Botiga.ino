//PROGRAMA PER FER FUNCIONAR LES BARRERES OPTIQUES a base de micro ProMINI
//CINTA BOTIGA... 2 TIRES

//1.0 es controla una barrera d'un element, i s'ha constatat el funcionament correcte del byte resultant dels diodes encesos i apagats.
// ATENCIÓ  : bit = 1   barrera lliure.  Bit = 0 barrera obstruida
//no funciona el control de dos elements, es deixa per properes versions

//funciona perfectament, provat en totes les necessitats el 3/10. versio 1.3. copiat a versio 1.5 exacte



# define prova 13     // provisional
# define Entrada_On_Off 12     // Entrada de Posar en Marxa, o Apagar
# define Entrada_Leds   A1     // Entrada Multiplexada de estat dels fotocaptors
#define Control_485_1 2     //0=recepcio RX  1= transmissio TX   Control de flux RS485
# define FLAG1           3     // Control del Flag 1, posicionat al numero de led
# define FLAG2           4     // Control del Flag 2, posicionat al numero de led
#define OFFTotal  11    // quan aquesta sortida es posa a 'o', parem els leds de funcionar.
# define SortidaA       10     // 1 Canal A BCD
# define SortidaB        5     // 2 Canal A BCD
# define SortidaC        6     // 4 Canal A BCD

# define SortidaD        7     // 1 Canal B BCD
# define SortidaE        8     // 2 Canal B BCD
# define SortidaF        9     // 4 Canal B BCD

//________________________________ Variables COMUNICACIO ________________________________
unsigned long Milis30 = 0;
unsigned long OldMilis30 = 0;
bool EnvioTrama = false ;  //EnvioTrama = 1 funcionament en TX
unsigned int crc = 0xFFFF ;
byte TramaEnvio[14] ;
char TramaChar[14] ;
char TramaChar1[14] ;
byte TramaCalculCrc[14] = {};
byte TramaAutoritzada[14] ;
byte TramaRecepcio[14] = {} ;
byte crcBaixCalculat = 0x00;
byte crcAltCalculat  = 0x00;
byte crcBaixLlegit = 0x00;
byte crcAltLlegit  = 0x00;
byte ByteControl = 0;
byte ByteConsola = 0;
byte OrdreAExecutar = 0;
bool OrdreDeRecepcio = false ;
//________________________________   Gestio Trama Rebuda   i ordres generals________________________________
bool PreguntaFlag = false;
int NumMissatgeRebut = 0;
int NumMissatgeEnviar = 0;
bool EnviarEstat = false ;
bool FlagByte1 =  false;
bool FlagByte2 =  false ;
bool MissatgeOKRebut =  false ;
bool MissatgeRebut =  false ;
bool SistemaMarxa = false;    //  quan aquesta variable està a 0 el sistema de leds no funciona (paro ordenat per Master) 

//  CONSOLA______________________________________________
String Consola ;
int NivellGravatManual = 0;
int OldFase = 0 ;
int Fase = 0;      //conmutador de les diferents fases del funcioname
bool MostraPantalla = 0;     
//____________________________________________
int ComRecepcions = 0; //comptador  
bool Llegit = false;  // indica que hem llegit una trama amb succes
bool CrcCalculOk = false;
byte AdrePlaca ='v';   // v DE CINTA BOTIGA                                                                 ▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// byte Adre2Placa = 0x73;  //s                                                                                          ▄▄▄▄▄▄▄▄▄▄▄▄▄▄
byte AdreTots = 'T';   //T de Tots                                                                                      ▄▄▄▄▄▄▄▄▄▄▄▄▄▄
byte NombreDePlaques = 2;   //els nombre de parells de plaques que té el conjunt  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄
bool ComAcceptada = false;
int CT = 0; //comptador
int ComptCalculOr = 0 ;
bool HeRebut = false ;  //indicador de recepció de trama correcta a explotar  
bool ModoProva = false;
bool CanviTira = false;
//________________________________   Strings Comunicacio ________________________________
byte TramaFLAGpregACarro[14] = {'A','B','A','P','R','E','F',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA CARRO  atenció s'ha de canviar el receptor (adreça de placa ) segona lletra per cada programa
byte TramaFLAGOrdACarro[14] = {'A','B','A','O','R','D','A',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ; 
byte TramaFLAGpregACintMag[14] =                       {'A','m','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;  //DE MASTER A BARRERA DE CINTA MAGATZEM
byte TramaFLAGpregACintVen[14] =                       {'A','V','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA DE CINTA VENEDORS
byte TramaFLAGpregATobMag[14] = {'A','Z','A','P','R','E','F',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA TOBOGAN MAGATZEM
byte TramaFLAGpregATobBotiga[14] =                    {'A','X','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;    //DE MASTER A BARRERA TOBOGAN BOTIGA
byte TramaFLAGpregATots[14] =                              {'A','T','A','F','L','A','?',0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;   //DE MASTER A BARRERA TOTS 

byte TramaFLAGrespDeMobBase[14] = {'A','A','B','C','O','N','f',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE MOBIL  BARRERA  A MASTER
byte TramaFLAGrespDeMob[14];
byte TramaFLAGrespDeCinMag[14] =   {'A','A','Z','C','O','N','f',0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //  DE CINTA MAGATZEM BARRERA  A MASTER
byte TramaFLAGrespDeCinVen[14] =    {'A','A','v','C','O','N','f',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE CINTA VENEDORS BARRERA A MASTER 
byte TramaFLAGrespTobMag[14] =    {'A','A','Z','C','O','N','f',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE TOBOGAN MAGATZEM  BARRERA  A MASTER 
byte TramaFLAGrespTobBotiga[14] =                  {'A','A','X','F','L','A','G',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE TOBOGAN BOTIGA  BARRERA  A MASTER 
byte TramaResposta[14];
byte TramaOK[14] = {'A','T','A','O','K',0x00,'<',0x00,0x00,0x00,0x00,0x00,0x00,0x00};    //  DE TOBOGAN BOTIGA  BARRERA  A MASTER 
byte TramaFLAGresposta[14];
//________________________________ Variables ________________________________
bool OnOff = 0;
int NumeroDeLed;              // Variable que Selecciona el Numero de Led  amb BCD
bool AccNumeroDeLed = false; 
byte OldNumeroDeLed;
byte NumeroDeTira;             // Variable que Selecciona el Numero de Tira amb BCD
byte EntradaMultiplexatLeds1;   // Variable on si entrodueix el estat dels leds
byte EntradaMultiplexatLeds2;   // Variable on si entrodueix el estat dels leds
byte EntradaMultiplexatLeds3;
byte M0= 0;                       // Memoria 0
bool A;                        // Sortida Boolean A BCD ( Numero de Led )
bool B;                        // Sortida Boolean B BCD ( Numero de Led )
bool C;                        // Sortida Boolean C BCD ( Numero de Led )
bool D;                        // Sortida Boolean D BCD ( Numero de Tira )
bool E;                        // Sortida Boolean E BCD ( Numero de Tira )
bool F;                        // Sortida Boolean F BCD ( Numero de Tira )
bool G;                        // Sortida Boolean G BCD ( Numero de Tira )

bool Flag1 = 0 ;
bool Flag2 = 0 ;
bool EntradaDigital;           // Entrada Digital Passada a Boolean o 0 o be 1
int  Entrada;                  // Variable on rebem els valors rebuts de la  Entrada analogica ( Entrada_Leds )
int FlagByte;

bool FinalLeds = false ;
bool SequenciaNum = false ;
bool AccesTira = false;

int Coincidencia = 0;
bool PROVA = false;
//________________________________ Variables  de rellotge i temps  ________________________________
unsigned long CentMilisegons;
unsigned long CincCentsMicros;     // Variable de Temps Actual   per fer el rellotge       
unsigned long SisCentsMicros;   // Variable de Temps Anterior per fer el rellotge
unsigned long DosCentMicros;
unsigned long MicroSegons;
unsigned long UnSegon;
bool TopSegon;
bool TopCentMilis;
unsigned long Segons;
bool PostaAZero= false ;
bool Top = true ;
bool Top1 = true ;
bool Top2 = true ;
bool ControlTop1;
bool ControlTop2;
int ComptadorPerEnvios;

#include <avr/wdt.h> 
#define RESETWATCHDOG
#include <EEPROM.h> 
#include <SoftwareSerial.h>

void setup(){          //________________________________ Void Setup ________________________________

pinMode(Control_485_1,OUTPUT);        // Sortida del control de flux del RS485    
digitalWrite(Control_485_1,HIGH);       // Reiniciarla en Recepcio

pinMode(Entrada_Leds, INPUT);  // Entrada de la lectura dels leds
pinMode(FLAG1, OUTPUT);         // Sortida Flag 1
pinMode(prova, OUTPUT); 
pinMode(FLAG2, OUTPUT); // Sortida Flag 2
pinMode(OFFTotal, OUTPUT);    //per parar el funcionament dels leds.
pinMode(SortidaA, OUTPUT);     // Sortida Fisica BCD A ( Numero de Led )
pinMode(SortidaB, OUTPUT);     // Sortida Fisica BCD B ( Numero de Led )
pinMode(SortidaC, OUTPUT);     // Sortida Fisica BCD C ( Numero de Led )

pinMode(SortidaD, OUTPUT);     // Sortida Fisica BCD D ( Numero de Tira )
pinMode(SortidaE, OUTPUT);     // Sortida Fisica BCD D ( Numero de Tira )
pinMode(SortidaF, OUTPUT);     // Sortida Fisica BCD D ( Numero de Tira )


Serial.begin(9600);            // Obrir el port de comunicacions a 9600 baudis
Serial.setTimeout(40); 
 wdt_disable();
 wdt_enable(WDTO_8S); 
delay(5000);
SistemaMarxa = true;    // mentre no comuniquem els ordres   de posar en maerxa i parar
digitalWrite ( OFFTotal, HIGH);
}


void loop() {  //█  LOOP  █████████████████████████████████████████████████████████████
   //███████████████████████████████████████████████████████████████████████
 wdt_reset();


//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄      Gestio del Temps     ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
Top = false;
Top1 = false;
Top2 = false;
TopSegon = false;
MicroSegons=micros();
  if ( CentMilisegons + 100000 <= MicroSegons ) {  TopCentMilis = true;  CentMilisegons = micros();  } 
 if ( CincCentsMicros + 1200 <= MicroSegons ) {  Top = true; ControlTop1 = true ;  ControlTop2 = true ; CincCentsMicros = micros();  } //durada en us del cicle total d'un led
  if ( ( CincCentsMicros +300 <= MicroSegons ) && ControlTop1 == true ) {  Top1 = true;    ControlTop1 = false ;  }  // punt de medició . s'ha de tenir en compte que lect analogica dura 160us.
  if ( ( CincCentsMicros + 850 <= MicroSegons ) && ControlTop2 == true) {  Top2 = true;  ControlTop2 = false ;   }// moment de parada per no generar un consum constant llarg per les alimentacions
if (UnSegon + 1000000 <= MicroSegons ) { TopSegon = true; Segons++; UnSegon = MicroSegons ; PostaAZero = false ; }
                           if ( MicroSegons < 4000 && PostaAZero == false ) 
                           { CincCentsMicros = 0 ; SisCentsMicros = 0;  DosCentMicros = 0 ; UnSegon = 0; CentMilisegons = 0;
                           PostaAZero = true ;  Milis30 = 0  ; OldMilis30 = 0;  }  //posta a '0' per overflow
                           
 //▄▄▄▄▄▄▄▄▄▄▄  control durada cicle en la pota 13  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄                           
// PROVA = !PROVA;    //per controlar la durada d'un cicle sencer... s'estipula en 160 a 200us normalment i en la fase de recepcio/emissio dura 10ms
// digitalWrite( prova,PROVA );          
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 

  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   Gestio Sequencia Leds   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
 //anem fent la sequencia dels 8 leds de cada placa, i anem a sequencia placa per avançar cap a la seguent 
 //només es pararà durant la recepci i reemissio, lo que dura uns 10ms
 //els Top els mana el rellotge
  if (Top == true  && HeRebut == false )  {                                     
                                    NumeroDeLed++ ;                            // comptador de leds, es el comptador base que es fa servir per 
                                        if ( AccNumeroDeLed == true ) {     // aquesta variable puja quan sobrepassem el led 7, i hem d'anar doncs al led 0 de la placa seguent
                                                                                              NumeroDeLed = 0; 
                                                                                              AccNumeroDeLed =  false ;                                                                                              
                                                                                               NumeroDeTira++;    if ( NumeroDeTira > NombreDePlaques ) {  NumeroDeTira = 1;  }
                                                                                               SequenciaPlaca();
                                                                                               SequenciaLed(); 
                                                                                              
                                                                                               }   //passem de 7 a 0 : 8 leds                                                                                                              
      SequenciaPlaca();  SequenciaLed();     }                    //anem al void de leds
if ( Top1 == true   && HeRebut == false  )     { SequenciaLed();    }                    //anem a efectuar la medicio                        
if ( Top2 ==   true  && HeRebut == false )     { SequenciaLed();    }                    //apaguem el led
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
 

//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   Gestio Trama Rebuda   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//aqui s'explora la trama rebuda, i es configura la resposta 

if (HeRebut == true ) {   //es fa servir AccNumeroDeLed perquè la comunicació coincideixi amb el final d'una exploració d'una placa. 
                                                                                                                     
                    if  ( TramaAutoritzada[6] == 'F')   {     for (int CT = 0; CT< 12 ; CT++ ){  TramaResposta[CT]  =  TramaFLAGrespDeCinVen[CT] ;}
                                                         
                                                                                          NumMissatgeRebut = TramaAutoritzada[9];
                                                                                          OrdreAExecutar =  TramaAutoritzada[6];     //que ens demanen ??                                                                                          

                                                       switch( OrdreAExecutar) {
                                                                                  case 'F':    TramaResposta[6] = 'f';    //enviarem flags
                                                                                                    break;
                                                                                  case 'M':   SistemaMarxa = false;      //parem en pausa
                                                                                                    digitalWrite ( OFFTotal, SistemaMarxa);  
                                                                                                    TramaResposta[6] = 'm';
                                                                                                    break;
                                                                                  case 'N':    SistemaMarxa = false;      //paro toral definitiu, es necessitara reset per tornar
                                                                                                    digitalWrite ( OFFTotal, SistemaMarxa);  
                                                                                                    TramaResposta[6] = 'n';
                                                                                                    break;
                                                                                  case 'A':    SistemaMarxa = true;          //posar en marxa sistema
                                                                                                     TramaResposta[6] = 'a';
                                                                                                    digitalWrite ( OFFTotal, SistemaMarxa);  
                                                                                                    break;
                                                                                  case 'B':    SistemaMarxa = true;           //posar en marxa sistema
                                                                                                     TramaResposta[6] = 'b';
                                                                                                     digitalWrite ( OFFTotal, SistemaMarxa);  
                                                                                                     break;         
                                                                                                  }                                                                                                 
                                                                                                  NumMissatgeEnviar++;                                                                                                  
                                                                                                  TramaResposta[1]   = TramaAutoritzada[2];
                                                                                                  TramaResposta[2]   = AdrePlaca ;
                                                                                                  bitWrite ( FlagByte,0,Flag1); bitWrite ( FlagByte,1,Flag2);
                                                                                                  TramaResposta[7]   = FlagByte ;
                                                                                                  TramaResposta[8]   = NumMissatgeEnviar ;
                                                                                                  TramaResposta[9]   = NumMissatgeRebut ;                                                                                                  
                                                                                                  TramaResposta[10] = EntradaMultiplexatLeds1;
                                                                                                  TramaResposta[11] = EntradaMultiplexatLeds2;

                                                                              EnviarEstat = true ;   // ordena la resposta
                                                                           }
                                                                       
 //_______________________________    resposta    ___________________________________________________________________

if (EnviarEstat == true){   
             for ( int CT = 0 ; CT < 12; CT++ ) { TramaEnvio[CT] = TramaResposta[CT] ; TramaResposta[CT] = 0; TramaAutoritzada[CT] = 0;}     
             EnvioTrama = true  ;
             TX485();        // envio de la trrama escollida coma resposta                         
 
             HeRebut     = false;  
             EnviarEstat = false ;
             OrdreDeRecepcio = false ;
                                     }
              AccNumeroDeLed = true; NumeroDeTira = 0;   // posem a '0' el comptador de leds, per a que no hi hagi males interpretacions         
                           }

 //▄▄▄▄▄   Flags   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄                             
//                             FLAGS           ██████████   MODIFICAR AQUEST APARTAT EN FUNCIÓ DE NECESSITATS DE FLAGS   ██████████
                                                                     
                               Flag1 = bitRead ( EntradaMultiplexatLeds1, 0 ); digitalWrite ( FLAG1,Flag1 );   //activem els flags que arriben directament a Master
                               Flag2 = bitRead ( EntradaMultiplexatLeds1, 1 ); digitalWrite ( FLAG2,Flag2 );                                                 
                            //    Flag3 = bitRead ( EntradaMultiplexatLeds3, 1 ); //digitalWrite ( FLAG3,Flag3 );

                               
 //▄▄▄▄▄▄   Anem  de recepcio   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄     
     RX485(); 
//▄▄▄▄▄▄   Tempo del port de sortida   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
//tancament del port d'envio un temps després de l'inici de l'envio, per a deixar-li el temps de que es descarregui de manera autònoma
                  Milis30 = millis();
                  if ((Milis30 - OldMilis30 ) > 20 ) { digitalWrite (Control_485_1, LOW);   }
                   
}//█  loop  ███████████████████████████████████████████████████████████████████
 //███████████████████████████████████████████████████████████████████████
//no utilitzem aquesta funcio perque perturba molt la commutacio dels leds.
/*void ImprimirConsola(){                  // imprimim en pantalla per controlar funcionament del sistema de leds
 Serial.print("Flag1:  "); Serial.print(Flag1); Serial.print("   Flag2:  "); Serial.print(Flag2);  
  Serial.print("▄▄  Tira 1:  "); Serial.print(EntradaMultiplexatLeds1,BIN); Serial.print("  Tira 2:  "); Serial.print(EntradaMultiplexatLeds2,BIN);Serial.println(" ");
 // Serial.print("  Tira 3:  "); Serial.print(EntradaMultiplexatLeds3,BIN); Serial.println(" ");Serial.println(" ");       }                                                                                              
*/ 
void SequenciaPlaca(){   //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ Gestio de plaques  ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄      
//aqui escollim la placa que hem de posar en marxa, tant la d'emissio com la de recepció 
 
D = bitRead( NumeroDeTira,0  );                              // Llegir el bit 0, de la Variable numero de Tira, i passar-ho al boolean D ( Que es una sortida de un pin BCD )
E = bitRead( NumeroDeTira,1  );                              // Llegir el bit 1, de la Variable numero de Tira, i passar-ho al boolean E ( Que es una sortida de un pin BCD )
F = bitRead( NumeroDeTira,2  );                              // Llegir el bit 2, de la Variable numero de Tira, i passar-ho al boolean F ( Que es una sortida de un pin BCD )

digitalWrite (SortidaD,D  );      // Passa del Boolean D, a la sortida fisica D
digitalWrite (SortidaE,E  );      // Passa del Boolean E, a la sortida fisica E
digitalWrite (SortidaF,F  );       // Passa del Boolean F, a la sortida fisica F
   
} //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 


void SequenciaLed() {   //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ Gestio Secuencia Leds ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
  // en aquesta seqüència anem encenent els leds un després de l'altre, i tenim cura d'apagar un temps el led abans d'encendre'n un altre
  //els Top estan ordenats per la gestio del temps...
  // en Top escollim el led a encendre de la placa corresponent. Una després de l'altra  !!
  // en Top1 llegim el Sensor infraroig que correspon al led encés, i ho carreguem en el byte EntradaMultiplexatLeds corresponent a la placa.
  //en Top2 posem diodes i plaques a 0, per deixar descansar l'alimentació, perquè es recuperi, ja que la intensitat dels leds es elevada.
  //a demés en placa 0 tenim tot apagat, perquè poguem apagar el sistema.
  //quan Top2, que és el final de cicle d'encesa d'un Led, coincidei amb Numerode Led = 7 aixequem una variable que ens posara poseriorment a '0', en el proper
  //Top passarem a '0' , canviarem de placa, i començara un nou cicle
  
if (Top == true ) {              
                              A = bitRead(NumeroDeLed,0);    // Llegir el bit 0, de la Variable numero de led, i passar-ho al boolean A ( Que es una sortida de un pin BCD )
                              B = bitRead(NumeroDeLed,1);    // Llegir el bit 1, de la Variable numero de led, i passar-ho al boolean A ( Que es una sortida de un pin BCD )
                              C = bitRead(NumeroDeLed,2);    // Llegir el bit 2, de la Variable numero de led, i passar-ho al boolean A ( Que es una sortida de un pin BCD )
                                                         
                              digitalWrite(SortidaA,A);      // Passa del Boolean A, a la sortida fisica A  això encén el led corresponent.
                              digitalWrite(SortidaB,B);      // Passa del Boolean B, a la sortida fisica B  això encén el led corresponent.
                              digitalWrite(SortidaC,C);      // Passa del Boolean C, a la sortida fisica C això encén el led corresponent.

                          }
if (Top1 == true ) { Entrada = analogRead ( Entrada_Leds );
                             if  (   Entrada >350) {   EntradaDigital = true;   }  else   {   EntradaDigital = false;   }
                             switch (NumeroDeTira ) {
                              case 1:  bitWrite(EntradaMultiplexatLeds1,NumeroDeLed,EntradaDigital);  break;
                              case 2:  bitWrite(EntradaMultiplexatLeds2,NumeroDeLed,EntradaDigital);  break;
                              case 3:  bitWrite(EntradaMultiplexatLeds3,NumeroDeLed,EntradaDigital);  break;
                                                                     }
                             }
                             
if (Top2 == true )  {  digitalWrite (SortidaA, LOW  ); digitalWrite (SortidaB,LOW  );  digitalWrite (SortidaC,LOW  ); 
                                 digitalWrite (SortidaD,LOW  ); digitalWrite (SortidaE,LOW );  digitalWrite (SortidaF,LOW );  
                              if (NumeroDeLed == 7 )  {  AccNumeroDeLed = true;  } //quan hem fet el Led 7 canviem de placa i/o tornem a començar amb el '0'
                              }            
}//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 


//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄   EMISSIO TX 485      ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// aquest subprograma envia una trama "  TramaEnvio[]   " de 14 bytes. en els dos últimsTramaEnvio[12]  i TramaEnvio[13] la trama col.loca el CRC, i la envia tota
//els codis d'aquesta trama estan establerts en l'Excel  : D:\Imaginem\IMA_TECNIC\Projectes\__IMAGINEM PROJECTES\IMA ANALOGIC CONTROL\LLENGUATGE 485 GENERACIO 2019\LLENGUATGE485
//les configuracions inicials es troben en el fitxer adjunt explicacions
//s'ha de tenir compte per quin canal serie s'envien les trames, i canviar-lo si cal  " Serial.write (TramaEnvio[CT4]) ; Serial1.write (TramaEnvio[CT4]) ; Serial2.write (TramaEnvio[CT4]) ;  Serial3.write (TramaEnvio[CT4]) ; "

void TX485() {
//___012____CALCUL CRC I CARREGA EN LA TRAMA D'ENVIO_________________________________________________________________________________                                                                   


           if (EnvioTrama == true )  {                                                                                                                                    //calculs del CRC
                             crc = 0xFFFF ;
                             for ( int CT = 0 ; CT < 12; CT++ ){ //ElementsTrama
                             crc = crc   ^=   TramaEnvio[CT];      
                             for (int CT11 = 8 ; CT11  != 0 ; CT11--) 
                                    { if ((crc & 0x0001) !=0 ) {    crc = crc >> 1;   crc = crc ^= 0xA001;   }  else { crc = crc >> 1;} }    }                          
                                    crcBaixCalculat = crc & 0xFFFF;       crcAltCalculat =  crc >>8;   
                 TramaEnvio[12] = crcBaixCalculat; TramaEnvio[13] = crcAltCalculat;                                                               //una vegada calculats els CRC els posem en a trama que sera enviada

//___014  envio COMUNICACIO   ___forma part del 012______________________________________________________________________________
                    delay(6);   
                    digitalWrite (Control_485_1, HIGH); OldMilis30 = millis();  delay (1);                       //obre la sortida 485 a TX, que es tancara després de 35 ms
                                           Serial.write (0x3A) ; delay(1);                                                                                                      //posem primer el byte BB per a informar de principi de trama
                                           for (int CT4 = 0; CT4 <14 ; CT4++)   { Serial.write (TramaEnvio[CT4]) ;  TramaChar[CT4] = TramaEnvio[CT4];  }  //envio de trama i posta a 0 de la trama per a evitar errors        TramaEnvio[CT4] = 0                                    
                                            EnvioTrama = false;                                                                                                                    //tanquem l'envio de trama                                                                                                                     
                                                     }   
}  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 

void RX485() {  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ RECEPCIO RX 485 ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//en aquest subprograma es reb una trama per RX TX o 485, documentada amb CRC. una vegada autoritzada i controlada la autenticitat de l'envio gràcies al CRC, la trama sortint té
//la referència "  TramaAutoritzada[CT]  ", si es vol treballar sota forma de bytes, decimal, binari, o bé té la forma "  TramaChar[CT]  " si es vol utilitzar sota forma de caràcters.

//unsigned long NovaCoordinada = 0;
//byte NivellActual = 0;    //nivell al qual estem
//byte NivellDesitjat = 0;


//||||||||||||||||||||||||||||||||||||||||||||||||||||  RECEPCIO  ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||5||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
 //___020___ Recepcio d'una trama de 14 bytes, incloent 2bytes de CRC._precedentment rebem un byte d'identificacio 0x0B________________________________________________________

if (Serial.available() >14  )  {   ByteControl = Serial.read();                                           
                                           if (ByteControl == 0x3A) { ComRecepcions++; Llegit = 1;     //Serial.println ( '2') ;
                                           for ( int CT6 = 0; CT6 <14;  CT6++) { TramaRecepcio[CT6] = Serial.read(); TramaChar1[CT6] = TramaRecepcio[CT6];  }  //Serial.print  ( TramaChar1[CT6]) ; 
                                          // digitalWrite (SortidaD,LOW  ); digitalWrite (SortidaE,LOW  );  digitalWrite (SortidaF,LOW  );  delay(1000);
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
//_____022____FI__________________________________________________________________________________________________________     
//___024___Calcul CRC ___________________________________________________________________________________________________
                                 crc = 0xFFFF ;
                                 for ( CT = 0 ; CT < 12; CT++ ){ //ElementsTrama
                                 crc = crc   ^=   TramaCalculCrc [CT];      
                                 for (ComptCalculOr = 8 ; ComptCalculOr  != 0 ; ComptCalculOr--) 
                                            { if ((crc & 0x0001) !=0 ) {    crc = crc >> 1;   crc = crc ^= 0xA001;   }  else { crc = crc >> 1;} }    }                          
                                 crcBaixCalculat = crc & 0xFFFF;       crcAltCalculat =  crc >>8;                   
//______024___FI_____________________________________________________________________________________________________                   
//___026___autorització de presa en compte de trama_____________________________________________________________________

                                if ( crcAltCalculat == crcAltLlegit &&  crcBaixLlegit== crcBaixCalculat)       //la trama passa a ser acceptada quan els CRC corresponen
                                       {   CrcCalculOk = true;   HeRebut = true;     //   comptadorOKs++;                 
                                           for (int CT = 0; CT <14; CT ++) { TramaAutoritzada[CT] = TramaCalculCrc[CT];  TramaChar[CT] = TramaCalculCrc[CT]; }
                                        //   for (int CT = 0; CT <6; CT ++) {      //envio trama petita de aceptació
 // Serial.print( "M     :   ") ; Serial.print ( crcAltCalculat) ;Serial.print ( " ") ;Serial.print ( crcAltLlegit ) ;Serial.print ( " ") ;Serial.print (  crcBaixCalculat ) ; Serial.print ( " ") ;Serial.println ( crcBaixLlegit ) ; 
                                                                                            }                                                                                                                              
            }  else  {  HeRebut = false;    for (CT = 0; CT <14; CT++) { TramaCalculCrc[CT] = 0;TramaAutoritzada[CT] = 0; TramaRecepcio[CT] = 0; } }

}   //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ 
/*
void RecepcioConsola(){  //▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄ RECEPCIO CONSOLA ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//||||||||||||||||||||||||||||||||||||||||||||||||||||  RECEPCIO  PER CONSOLA||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||5||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
if ( Serial.available() > 0 )   { Consola = Serial.readString();  NivellGravatManual  = Consola.toInt() ;
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
                                  }
  */                                
