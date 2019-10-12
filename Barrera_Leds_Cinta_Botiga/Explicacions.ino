/*
ADRECES
BYTE 0 CLIENT VILADOMAT = CLIENT A
BYTE1
S     0x53  PLACA  DRIVER SHUTTLE   
C     0x43   PLACA DRIVER MASTER brush MOBIL
E      0x45    PLACA DRIVER DC MOBIL
M    0x4D   PLACA DRIVER CINTA  MAGATZEM
V     0x56    PLACA DRIVE CINTA VENEDORS
B     0x42     BARRERA CINTA CARRO MOBIL
Z     0x5A      BARRERA  TOBOGAN MAGATZEM
m    0x6D    BARRERA CINTA  BAIX  =  m minúscula
v     0x76     BARRERA CINTA  BOTIGA   =    v minúscula
X     0x78     BARRERA TOBOGAN BOTIGA  1/2/3 GRAN 
T     0x54     A TOTS 
W   0X57   A TOTS ELS MOTORS 

0    Coordinada a baix de tot
1    base per fer els 0  20.000 
2     alarma baix  només es pot transpassar a velocitat reduida
3    tècnica de treball en magatzem
4    de sortida del material cap al magatzem tobogan
5    de descarrega de materia en cinta de magatzem(baixada)
6    de carrega de material per la cinta de magatzem(pujada)
7    de tapar forat en cas d'incendi
8    de recollida caixes en cinta venedors (baixada)
9    d'entrega de caixes per la cinta de venedors (pujada)
10   cota tècnica de treball en planta 0
11    tobogan primer
12   tobogan segon
13    tobogan tercer
14      alarma alt   només es pot traspassar a velocitat reduida
15    màxima abans d'explosió total

BYTE  2 ADREÇA EMISSOR
MISSATGES D'ERROR I INFORMACIO
BYTE 7    E = ERROR
                I = INFORMACIO
BYTE 8    L = FLAGS LLIURES
                O = FLAGS OCUPATS
                ES = CINTA INDICA A  CARRO UNA BONA SEPARACIÓ                                       DE  CAIXES (MATEIXA SAFATA O NO)
BYTE 9     C = TINC CAIXES EN CINTA
                 B = CINTA BUIDA

EXPLICACIO TASCA
ORDRES
ORD A - POSAR EN MARXA NORMAL AMB RESET
ORD B - POSAR EN MARXA NORMAL CONTINUACIÓ PROGRAMES
ORD C - 
ORD D - 
ORD E - 
ORD F - Enviar estat dels flags


ORD M - PARO TOTAL AMB RESERVA DE MEMORIES
ORD N - PARO TOTAL DEFINITIU

PREGUNTES
PRE O - 
PRE P - 
PRE Q - 
PRE R - 
PRE S - 


PRE Y - 
PRE Z - 

RESPOSTA PERQUÈ S'HA EXECUTAT
CON a -  Hem fet reset i posat en marxa sistema
CON b -  Hem posat en marxa sistema conservant memories
CON c - 
CON d - 
CON f -  T'envio l'estat dels flags de les meves cel.lules


CON m -  he parat funcionament en pausa
"CON n -   he parat funcionament definitiu. S'ha de fer Reset  
                 per tornar a funcionar"

RESPOSTA DE CONFIRMACIÓ DE RECEPCIÓ
CON o - 
CON p - 
CON q - 
CON r - 


CON x - 
CON y - 
CON z - 
*/
