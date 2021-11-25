#include "mbed.h"
#include <stdbool.h>

#define NUMBUTT             4
#define MAXLED              4
#define NUMBEAT             4
#define HEARBEATINTERVAL    100
#define IRBLACK1            3600    //numero entregao por infrarrojo1 cuando ve le linea
#define IRBLACK2            3600    //numero entregao por infrarrojo2 cuando ve le linea

//hola
typedef enum{ //contiene estados de la MEF //boton como pullup
    BUTTON_DOWN,    //0
    BUTTON_UP,      //1
    BUTTON_FALLING, //2
    BUTTON_RISING   //3
}_eButtonState;
_eButtonState myButton;//asignar estados a la MEF

typedef enum{
    IDLE=0,
    MODO1=1,
    MODO2=2,
    MODO3=3,
}e_modosA;
volatile e_modosA ModoAuto;

typedef enum{ //eventos del boton
    EV_PRESSED,
    EV_NOT_PRESSED,
    EV_NONE
}_eButtonEvent;

typedef struct{ //teclas
    _eButtonState estado;
    _eButtonEvent event;
    int32_t timeDown;
    int32_t timeDiff;
    uint32_t timeBoton;
}_sTeclas;
_sTeclas ourButton;//[NUMBUTT]

uint16_t mask[]={0x0001,0x0002,0x0004,0x0008}; // 0001 ,  0010,  0100,  1000

typedef enum{ //Enumeración de la MEF para decodificar el protocolo
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;
_eProtocolo estadoProtocolo;

typedef enum{ //Enumeración de la lista de comandos
        ACK=0x0D,
        ALIVE=0xF0,
        GET_LEDS=0xF1,
        SET_LEDS=0xF2,
        SERVO_ANG=0xA2,
        SONICO=0xA3,
        MOTORES=0xA1,
        HORQUILLA=0xA4,
        INFRARROJO=0xA0,
    }_eID;

typedef struct{              // Estructura de datos para el puerto serie
    uint8_t timeOut;         // TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t cheksumRx;       // Cheksumm RX
    uint8_t cheksumtx;       // Cheksumm Tx
    uint8_t indexWriteRx;    // Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     // Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    // Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     // Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   // Buffer circular de recepción
    uint8_t bufferTx[256];   // Buffer circular de transmisión
    uint8_t payload[32];     // Buffer para el Payload de datos recibidos
}_sDato ;
volatile _sDato datosComProtocol;

typedef union{ //campo de bits
    struct{
        uint8_t checkButtons :1;
        uint8_t Reserved :7; //7 bits restantes para futuras banderas
    }individualFlags;
    uint8_t allFlags;
}_bGeneralFlags;

volatile _bGeneralFlags myFlags;
uint8_t hearBeatEvent;

typedef union { //Unión para descomponer/componer datos mayores a 1 byte
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
}_udat;
_udat myWord;



void startMef();
void actuallizaMef();
/*void togleLed(uint8_t indice);*/
void manejadorLed(uint8_t mask);
void onDataRx(void);
void decodeProtocol(void);
void decodeData(void);   //decodifica los datos recividos por protocolo y manda datos
void encodeData(uint8_t auxID);   //codifica y manda datos por el protocolo
void sendData(void);
void OnTimeOut(void);
void hearbeatTask(void);
void ServoDriver();
void MotorDriver();   // se encarga de manejar los motores
void myTrigger();
void startEcho();           //Manda un eco con el sensor ultrasonico.
void endEcho();             //Recibe el eco del ultrasonico para medir distancia.
void VelocimetroR();        //Funcion para leer el sensor horquilla Right.
void VelocimetroL();        //Funcion para leer el sensor horquilla Left.
void infrarrojo();          //Funcion para leer el sensor infrarrojo.
void giroenU();             //Funcion para girar el auto 90 en una direccion o 180 grados.
void modo1();
void modo2();    
//void parpadealed(uint8_t); 


//BusIn buttonArray(PB_6,PB_7,PB_8,PB_9);
//BusOut leds(PB_1/*PB_12,PB_13,PB_14,PB_15*/);
BusIn Button(PB_4);                 //boton modos
DigitalOut dir1(PB_15);               //< 
DigitalOut dir2(PB_14);               //< direcciones de los motores
DigitalOut dir3(PB_7);                //<
DigitalOut dir4(PB_6);                //<
DigitalOut Trigger(PB_13);          //salida del trigger (sensorultrasonico)
InterruptIn UltraSonic(PB_12);      //pin del eco (sensorultrasonico). al ser un interrupt interrumpe (quien lo diria) el codigo al llegar señal al pin
InterruptIn TacometroR(PB_8);       //pin del sensor horquillaR (recibe pulso)
InterruptIn TacometroL(PB_9);       //pin del sensor horquillaL (recibe pulso)
AnalogIn InfrarrojoL(PA_0);         //pin del infrarrojo 1
AnalogIn InfrarrojoR(PA_1);         //pin del infrarrojo 2

PwmOut Servo(PB_10);                //salida del servo
PwmOut MotorA(PB_5);                //salida motor A left//5
PwmOut MotorB(PA_8);                //salida motor B right//8

DigitalOut HEARBEAT(PC_13);         // Defino la salida del led
Serial pcCom(PA_9,PA_10,115200);    // Configuración del puerto serie, pin de rx, de tx y velocidad 115200

Timer miTimer;                      // Timer general
Timer timerEco;                     // timer para el eco (sensorultrasonico)
Timeout ToutTrigger;                // time out para controlar el tiemo del trigger

Ticker timerGeneral;

typedef struct{
    uint32_t timeEco=0;                     //Guarda dato a mandar sensor sonico
    uint32_t timeSonico=0;                  //guarda tiempo del sensor sonico
    uint32_t timeIR=0;                      //guarda tiempo de infrarrojo
    uint32_t Velocity=0;                    //guarda el dato a mandar del sensor horquilla
    int8_t auxPosServo=0 ;                  //auxiliar global posicion del servo
    int8_t auxRPM;                          //Guarda RPM a mandar sensor Left
    int8_t auxRPM2;                         //Guarda RPM a mandar sensor Right
    volatile uint32_t timeVelocimetro=0;    //Saber cuando mandar datos a qt
    volatile uint32_t timeVelocimetro2=0;   //Saber cuando mandar datos a qt
    uint16_t tiksHorquilla=0;               //Contador del sonsor de horquilla para calculo de velocidad Left
    uint16_t tiksHorquilla2=0;              //Contador del sonsor de horquilla para calculo de velocidad Rigth
    uint16_t valueIR1=0;                    //Valor del sensor infrarrojo Left 
    uint16_t valueIR2=0;                    //Valor del sensor infrarrojo Right
    uint8_t auxCL=0;
    int8_t auxAng=0;
    uint8_t modoAuto=0;                     //modo delauto
    uint8_t Qmotor=0;                       //que motor girar
    uint8_t Qsentido=0;                     //en que sentido girar los motores 
    uint32_t velMot1=0;                     //velocidad motor1
    uint32_t velMot2=0;                     //velocidad motor2
    volatile uint8_t hearbeatevent2=0;      //usada en el hearbeat de los modos
    int hearbeatTime=0;                     //medir tiempo del hearbeat

}_sAux1;
volatile _sAux1 sAuxiliares;

typedef struct{
    volatile bool togleSonico=0;        //usada para saber si mandar datos de sonico a qt
    volatile bool togleVelocimetro=0;   //usada para saber si mandar datos de horquilla a qt
    volatile bool  pwmMotor=0;           //saber si apagar o prender motor motor
    volatile bool togleInfrarrojo=0;    //usada para saber si mandar datos de los infrarrojos a qt
    bool echosonico=0;
    bool ejecutarmodo=0;                //saber si se ejecuta el modo

}_sBanderas;
volatile _sBanderas sBande;

int main()                                                                          //////////////////
{
    //int hearbeatTime=0;
    miTimer.start();
    timerEco.start();
    myFlags.individualFlags.checkButtons=false;
    hearBeatEvent=0;
    pcCom.attach(&onDataRx,Serial::RxIrq);
    timerGeneral.attach_us(&OnTimeOut, 50000);

    UltraSonic.rise(&startEcho);    // flanco de subida del eco, llamo a la funcion inicia el timer
    UltraSonic.fall(&endEcho);      // flanco de bajada eco, llamo a la funcion mando datos del timer
    TacometroR.rise(&VelocimetroR);  // flanco de subida del horquillaR, llamo a la funcion para
    TacometroL.rise(&VelocimetroL);  // flanco de subida del horquillaL


    while(true){

        if ((miTimer.read_ms()-sAuxiliares.timeSonico)>=HEARBEATINTERVAL){ //entra cada 100ms
            myTrigger();
            ToutTrigger.attach_us(&myTrigger,10);
            sAuxiliares.timeSonico=miTimer.read_ms();
        }
        if ((miTimer.read_ms()-sAuxiliares.timeIR)>=40){
            infrarrojo();
            sAuxiliares.timeIR=miTimer.read_ms();
        }

        if(miTimer.read_ms()-ourButton.timeBoton>=40){
            ourButton.timeBoton=miTimer.read_ms();
            if(Button.read()){
                ourButton.event=EV_NOT_PRESSED;
            }else{
                ourButton.event = EV_PRESSED;
            }
            actuallizaMef();
            if (ourButton.timeDiff>100 && ourButton.timeDiff<1000){
                ourButton.timeDiff=0;
                sAuxiliares.modoAuto++;
                sBande.ejecutarmodo=0;
                if (sAuxiliares.modoAuto>=4){
                    sAuxiliares.modoAuto=0;
                }
            }else{
                if(ourButton.timeDiff>=1500 && ourButton.timeDiff<3000){
                    sBande.ejecutarmodo =! sBande.ejecutarmodo; //ejecuto modo
                }
            }
        }
        hearbeatTask();

        switch(sAuxiliares.modoAuto){
            case 0: //IDLE  <- luz parpadeante cada 100ms
                //en este modo se espera la ejecucion y cambio de modo
            break;
            case 1:
                if(sBande.ejecutarmodo){
                    //HEARBEAT=1;
                    modo1();
                }
            break;
            case 2:
                if(sBande.ejecutarmodo){
                    modo2();
                }
                //hola
            break;
            case 3:
                if(sBande.ejecutarmodo){
                    /* modo3(); */
                }
            break;

        }

        /*if ((miTimer.read_ms()-hearbeatTime)>=HEARBEATINTERVAL){
            hearbeatTime=miTimer.read_ms();
            hearbeatTask();
        }*/
        if(datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx) 
            decodeProtocol();

        if(datosComProtocol.indexReadTx!=datosComProtocol.indexWriteTx) 
            sendData();

    }
    return 0;
}

void startMef(){//MEF para DEBOUNCE de botones
   ourButton.estado=BUTTON_UP;
}

void actuallizaMef(){

    switch (ourButton.estado){
    case BUTTON_DOWN:
        if(ourButton.event){
           ourButton.estado=BUTTON_RISING;
        }
    break;
    case BUTTON_UP:
        if(!(ourButton.event)){
            ourButton.estado=BUTTON_FALLING;
        }
    break;
    case BUTTON_FALLING:
        if(!(ourButton.event)){
            ourButton.timeDown=miTimer.read_ms();
            ourButton.estado=BUTTON_DOWN;
            //Flanco de bajada
        }else{
            ourButton.estado=BUTTON_UP;    
        }
    break;
    case BUTTON_RISING:
        if(ourButton.event){
            ourButton.estado=BUTTON_UP;
            //Flanco de Subida
            ourButton.timeDiff=miTimer.read_ms()-ourButton.timeDown;
            //togleLed();
        }else{
            ourButton.estado=BUTTON_DOWN;
        }
    break;
    
    default:
        startMef();
        break;
    }
}

/************  Funciónes para manejar los LEDS ***********************/

/*void togleLed(uint8_t indice){
    uint16_t ledsAux=leds, auxmask=0;
    auxmask |= 1<<indice;
    if(auxmask & leds)
        ledsAux &= ~(1 << (indice)) ; 
    else
         ledsAux |= 1 << (indice) ;
    leds = ledsAux ;
}*/

/*void manejadorLed(uint8_t mask){
    uint16_t auxled=0, setLeds=0;
    auxled|=1<<3;
    if(auxled & mask)
        setLeds |= 1 <<3;
    else
        setLeds &= ~(1<<3);
    auxled=0;
    auxled|=1<<2;
    if(auxled & mask)
        setLeds |= 1 <<2;
    else
        setLeds &= ~(1<<2);
    auxled=0;
    auxled|=1<<1;
    if(auxled & mask)
        setLeds |= 1 <<1;
    else
        setLeds &= ~(1<<1);
    auxled=0;
    auxled|=1<<0;
    if(auxled & mask)
        setLeds |= 1 <<0;
    else
        setLeds &= ~(1<<0);

    leds=setLeds;
}*/

void decodeProtocol(void) //MEF para decodificar el protocolo serie
{
    static int8_t nBytes=0, indice=0;
    while (datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx)
    {
        switch (estadoProtocolo) {
            case START:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosComProtocol.cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosComProtocol.indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='R')
                estadoProtocolo=NBYTES;
            else{
                datosComProtocol.indexReadRx--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                nBytes=datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
               estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]==':'){
                   estadoProtocolo=PAYLOAD;
                    datosComProtocol.cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                    datosComProtocol.payload[0]=nBytes;
                    indice=1;
                }
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosComProtocol.payload[indice++]=datosComProtocol.bufferRx[datosComProtocol.indexReadRx];
                    datosComProtocol.cheksumRx ^= datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){
                    estadoProtocolo=START;
                    if(datosComProtocol.cheksumRx == datosComProtocol.bufferRx[datosComProtocol.indexReadRx]){
                        decodeData();
                    }
                }
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
    

}

void decodeData(void)//Función para procesar el comando recibido
{
    uint8_t auxBuffTx[50], indiceAux=0, cheksum ;
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (datosComProtocol.payload[1]) {
        case ALIVE:
            auxBuffTx[indiceAux++]=ALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            break;
            
        case GET_LEDS:
            auxBuffTx[indiceAux++]=GET_LEDS;
            //myWord.ui16[0]=leds;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[NBYTES]=0x04;
            break;

        case SET_LEDS:
            /*auxBuffTx[indiceAux++]=SET_LEDS;
            myWord.ui8[0]=datosComProtocol.payload[2];
            myWord.ui8[1]=datosComProtocol.payload[3];
            auxBuffTx[NBYTES]=0x02;
            manejadorLed(myWord.ui16[0]);*/
            break;

        case SERVO_ANG:
            auxBuffTx[indiceAux++]=SERVO_ANG;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            sAuxiliares.auxPosServo = datosComProtocol.payload[2];
            ServoDriver();
            break;

        case SONICO:
            auxBuffTx[indiceAux++]=SONICO;
            //auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            sBande.togleSonico=!sBande.togleSonico;
            break;

        case MOTORES: //cuando recibo la señal motores
            auxBuffTx[indiceAux++]=MOTORES;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x02;
            sBande.pwmMotor=!sBande.pwmMotor;
            if(sBande.pwmMotor==1){
                sAuxiliares.velMot2=1000;
                sAuxiliares.velMot1=1000;
            }else{
                sAuxiliares.velMot2=0;
                sAuxiliares.velMot1=0;
            }
            //sAuxiliares.Qsentido=1;
            //sAuxiliares.Qmotor=0x03;
            sAuxiliares.Qmotor=datosComProtocol.payload[2];
            sAuxiliares.Qsentido=datosComProtocol.payload[3];
            MotorDriver();
            break;
        case HORQUILLA://cuando recibo la señar de horquilla
            sAuxiliares.Velocity=1;
            
            
            break;
        case INFRARROJO://cuando recibo la señar del infrarrojo
            auxBuffTx[indiceAux++]=SERVO_ANG;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            sBande.togleInfrarrojo=!sBande.togleInfrarrojo;
            break;
        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
   for(uint8_t a=0 ;a < indiceAux ;a++){
       cheksum ^= auxBuffTx[a];
       datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
   }
    datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
}

void encodeData(uint8_t auxID)
{
    uint8_t auxBuffTx[50], indiceAux=0, cheksum ;
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (auxID){
        case ALIVE:
            auxBuffTx[indiceAux++]=ALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            break;

        case SONICO:
            auxBuffTx[indiceAux++]=SONICO;
            myWord.ui32=sAuxiliares.timeEco;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[indiceAux++]=myWord.ui8[2];
            auxBuffTx[indiceAux++]=myWord.ui8[3];
            auxBuffTx[NBYTES]=0x06;
            break;
        case HORQUILLA:
            auxBuffTx[indiceAux++]=HORQUILLA;
            myWord.ui32 = sAuxiliares.Velocity;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[indiceAux++]=myWord.ui8[2];
            auxBuffTx[indiceAux++]=myWord.ui8[3];
            auxBuffTx[NBYTES]=0x06;
            break;
        case INFRARROJO:
            auxBuffTx[indiceAux++]=INFRARROJO;
            myWord.ui16[0] = sAuxiliares.valueIR1;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            myWord.ui16[0] = sAuxiliares.valueIR2;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[NBYTES]=0x06;
            break;
        case 5://cambiar por el nombre del comando que se quiera agregar
            break;
        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
    cheksum=0;
    for(uint8_t a=0 ;a<indiceAux ;a++)
    {
        cheksum ^= auxBuffTx[a];
        datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
    }
        datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
}

void sendData(void) // Función para enviar los bytes hacia la pc
{
    if(pcCom.writable())
        pcCom.putc(datosComProtocol.bufferTx[datosComProtocol.indexReadTx++]);

}

void hearbeatTask(void) {  //Función para hacer el hearbeats
    
    switch (sAuxiliares.modoAuto){
    case 0:
        if ((miTimer.read_ms()-sAuxiliares.hearbeatTime)>=HEARBEATINTERVAL){
            sAuxiliares.hearbeatTime=miTimer.read_ms();
            HEARBEAT=!HEARBEAT;
        }
        break;
    case 1:
        if(sBande.ejecutarmodo == 0){ //Luz parpadeante cada 3 segundos
            if ((miTimer.read_ms()-sAuxiliares.hearbeatTime)>=100){
                sAuxiliares.hearbeatTime=miTimer.read_ms();
                if(sAuxiliares.hearbeatevent2==1){
                    HEARBEAT=0;
                }else{
                    HEARBEAT=1;
                }
                sAuxiliares.hearbeatevent2 = (sAuxiliares.hearbeatevent2>=30) ? (0) : (sAuxiliares.hearbeatevent2+1);
            }
        }else{
            if ((miTimer.read_ms()-sAuxiliares.hearbeatTime)>=100){
                sAuxiliares.hearbeatTime=miTimer.read_ms();
                if(sAuxiliares.hearbeatevent2<=5 || sAuxiliares.hearbeatevent2 == 7){
                    HEARBEAT=0;
                }else{
                    HEARBEAT=1;
                }
                sAuxiliares.hearbeatevent2 = (sAuxiliares.hearbeatevent2>=30) ? (0) : (sAuxiliares.hearbeatevent2+1);
            }
        }
        break;
    case 2:
        if(sBande.ejecutarmodo==0){ //Luz parpadeante cada 3 segundos
            if ((miTimer.read_ms()-sAuxiliares.hearbeatTime)>=100){
                sAuxiliares.hearbeatTime=miTimer.read_ms();
                if(sAuxiliares.hearbeatevent2 == 1 || sAuxiliares.hearbeatevent2 == 3){
                    HEARBEAT=0;
                }else{
                    HEARBEAT=1;
                }
                sAuxiliares.hearbeatevent2 = (sAuxiliares.hearbeatevent2>=30) ? (0) : (sAuxiliares.hearbeatevent2+1);
            }
        }else{
            if ((miTimer.read_ms()-sAuxiliares.hearbeatTime)>=100){
                sAuxiliares.hearbeatTime=miTimer.read_ms();
                if(sAuxiliares.hearbeatevent2 <= 5 || sAuxiliares.hearbeatevent2 == 7 || sAuxiliares.hearbeatevent2 == 9){
                    HEARBEAT=0;
                }else{
                    HEARBEAT=1;
                }
                sAuxiliares.hearbeatevent2 = (sAuxiliares.hearbeatevent2>=30) ? (0) : (sAuxiliares.hearbeatevent2+1);
            }
        }
        break;
    case 3:
        if(sBande.ejecutarmodo==0){ //Luz parpadeante cada 3 segundos
            if ((miTimer.read_ms()-sAuxiliares.hearbeatTime)>=100){
                sAuxiliares.hearbeatTime=miTimer.read_ms();
                if(sAuxiliares.hearbeatevent2 == 1 || sAuxiliares.hearbeatevent2 == 3 || sAuxiliares.hearbeatevent2 == 5){
                    HEARBEAT=0;
                }else{
                    HEARBEAT=1;
                }
                sAuxiliares.hearbeatevent2 = (sAuxiliares.hearbeatevent2>=30) ? (0) : (sAuxiliares.hearbeatevent2+1);
            }
        }else{
            if ((miTimer.read_ms()-sAuxiliares.hearbeatTime)>=100){
                sAuxiliares.hearbeatTime=miTimer.read_ms();
                if(sAuxiliares.hearbeatevent2 <= 5 || sAuxiliares.hearbeatevent2 == 7 || sAuxiliares.hearbeatevent2 == 9 || sAuxiliares.hearbeatevent2 == 11){
                    HEARBEAT=0;    
                }else{
                    HEARBEAT=1;
                }
                sAuxiliares.hearbeatevent2 = (sAuxiliares.hearbeatevent2>=30) ? (0) : (sAuxiliares.hearbeatevent2+1);
            }
        }
        break;
    default:
        break;
    }
    
    /*if(hearBeatEvent < NUMBEAT){
        HEARBEAT=!HEARBEAT;
        hearBeatEvent++;
    }else{
        HEARBEAT=1;
        hearBeatEvent = (hearBeatEvent>=25) ? (0) : (hearBeatEvent+1);
    }*/
    //hola
}

void onDataRx(void){
    while (pcCom.readable())
    {
        datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
    }
}

void OnTimeOut(void){
    if(!myFlags.individualFlags.checkButtons)
        myFlags.individualFlags.checkButtons=true;
}

void ServoDriver(void){
    Servo.period_ms(20);
    Servo.pulsewidth_us(1500+((sAuxiliares.auxPosServo*1000)/180));
}

void MotorDriver(){
    //uint8_t var1=0;
    //sBande.pwmMotor!=sBande.pwmMotor;
    if(sAuxiliares.velMot1==0 && sAuxiliares.velMot2==0){
        dir1=0;
        dir2=0;
        dir3=0;
        dir4=0;
        //sBande.pwmMotor=0;
    }
    MotorA.period_us(1000);
    MotorB.period_us(1000);
    
    switch (sAuxiliares.Qmotor){
        case 0x01://motor A
            if (sAuxiliares.Qsentido){
                dir1=0;
                dir2=1;
            }else{
                dir1=1;
                dir2=0;
            }
            MotorA.pulsewidth_us(sAuxiliares.velMot1);
            
            break;
        case 0x02: //motor B
            if (sAuxiliares.Qsentido){
                dir3=0;
                dir4=1;
            }else{
                dir3=1;
                dir4=0;
            }
            MotorB.pulsewidth_us(sAuxiliares.velMot2);
            break;
        case 0x03: //ambos motores
            if (sAuxiliares.Qsentido){
                dir1=0;
                dir2=1;
                dir3=1;
                dir4=0;
            }else{
                dir1=1;
                dir2=0;
                dir3=0;
                dir4=1;
            }
            MotorA.pulsewidth_us(sAuxiliares.velMot1);
            MotorB.pulsewidth_us(sAuxiliares.velMot2);
            break;
        default:
            break;
    }
}

void myTrigger(){
    Trigger=!Trigger;
}

void startEcho(){
    timerEco.reset();
    //hola
}

void endEcho(){
    sAuxiliares.timeEco = timerEco.read_us();
    //sBande.echosonico!=sBande.echosonico;
    if (sBande.togleSonico){
        encodeData(SONICO);
    }
    //encodeData(SONICO);
}

void VelocimetroR(void){

    sAuxiliares.tiksHorquilla++;
    if (miTimer.read_ms()-sAuxiliares.timeVelocimetro>=500){
        sAuxiliares.auxRPM = sAuxiliares.tiksHorquilla;
        if(sBande.togleVelocimetro){
            encodeData(HORQUILLA);
        }
        sAuxiliares.tiksHorquilla=0;
        sAuxiliares.timeVelocimetro=miTimer.read_ms();
    }
}

void VelocimetroL(void){

    sAuxiliares.tiksHorquilla2++;
    if (miTimer.read_ms()-sAuxiliares.timeVelocimetro2>=500){
        sAuxiliares.auxRPM2 = sAuxiliares.tiksHorquilla2;
        if(sBande.togleVelocimetro){
            encodeData(HORQUILLA);
        }
        sAuxiliares.tiksHorquilla2=0;
        sAuxiliares.timeVelocimetro2=miTimer.read_ms();
    }
}

void infrarrojo(){
    sAuxiliares.valueIR1=InfrarrojoL.read_u16();
    sAuxiliares.valueIR2=InfrarrojoR.read_u16();
    if(sBande.togleInfrarrojo){
        encodeData(INFRARROJO);
    }
}

/*void giroenU(){
    
}*/

void modo1(){
    uint8_t varmode=0;

    if(sAuxiliares.timeEco > 400 && sAuxiliares.timeEco < 1200){
        varmode=1;
    }else{
        if(sAuxiliares.timeEco < 400){
            varmode=2;
        }
    }
    if(sAuxiliares.timeEco > 1200){
        varmode=3;
    }

    switch(varmode){
        case 1 : //objeto alejado del auto
            if(sAuxiliares.timeEco > 480){
                sAuxiliares.velMot1=400;
                sAuxiliares.velMot2=400;
                sAuxiliares.Qsentido=1;
                sAuxiliares.Qmotor=0x03;
                MotorDriver();
            }else{
                sAuxiliares.velMot1=0;
                sAuxiliares.velMot2=0;
                MotorDriver();
            }
            break;
        case 2 : //objeto cerca del auto
            if(sAuxiliares.timeEco < 380){
                sAuxiliares.velMot1=400;
                sAuxiliares.velMot2=400;
                sAuxiliares.Qsentido=0;
                sAuxiliares.Qmotor=0x03;
                MotorDriver();
            }else{
                sAuxiliares.velMot1=0;
                sAuxiliares.velMot2=0;
                MotorDriver();
            }
            break;
        case 3 : //en busca del objeto
            //Angulo -9
            /*if(sAuxiliares.auxAng == 90){
                sAuxiliares.auxAng = -90;
            }else{
                sAuxiliares.auxAng=sAuxiliares.auxAng+30;
            }*/
            sAuxiliares.auxAng = (sAuxiliares.auxAng==90) ? (-90) : (sAuxiliares.auxAng+30);
            sAuxiliares.auxPosServo=sAuxiliares.auxAng;
            ServoDriver();

            if(sAuxiliares.timeEco <= 1200){
                sAuxiliares.auxPosServo=0;
                ServoDriver();
                if(sAuxiliares.auxAng < 0){
                    //motor izq adelante
                    MotorDriver();
                }else{
                    if(sAuxiliares.auxAng > 0){
                        MotorDriver();
                    }
                }
                if(sAuxiliares.timeEco <= 1200){
                    MotorDriver();
                }
            }
            break;
    }
}

void modo2(){
    uint8_t var=0;
    if(sAuxiliares.timeEco <= 1000){
        var=2;
        sAuxiliares.velMot1=0;
        sAuxiliares.velMot2=0;
        MotorDriver();
    }else{
        var=1;
    }
    switch(var){
            case 1 : //sigue la linea

                /*if(sAuxiliares.valueIR1 < 50 && sAuxiliares.valueIR2 < 50){
                    MotorDriver(0x03,1,800);
                }else{
                    if(sAuxiliares.valueIR1>50){
                        MotorDriver(0x02,1,550);
                    }
                    if(sAuxiliares.valueIR2>50){
                        MotorDriver(0x01,1,550);
                    }
                }*/

                if(sAuxiliares.valueIR1 > IRBLACK1){
                    sAuxiliares.velMot1=500;
                    sAuxiliares.velMot2=300;
                    sAuxiliares.Qsentido=1;
                    sAuxiliares.Qmotor=0x03;
                    MotorDriver();
                }else{
                    sAuxiliares.velMot1=500;
                    sAuxiliares.velMot2=500;
                    sAuxiliares.Qsentido=1;
                    sAuxiliares.Qmotor=0x03;
                    MotorDriver();
                }
                if(sAuxiliares.valueIR2 > IRBLACK2){
                    sAuxiliares.velMot2=500;
                    sAuxiliares.velMot1=350;
                    sAuxiliares.Qsentido=1;
                    sAuxiliares.Qmotor=0x03;
                    MotorDriver();
                }else{
                    sAuxiliares.velMot2=500;
                    sAuxiliares.velMot1=500;
                    sAuxiliares.Qsentido=1;
                    sAuxiliares.Qmotor=0x03;
                    MotorDriver();
                }

                break;
            case 2 : //esquiva el objeto

                //giroenU();//gira90 a la izquierda
                sAuxiliares.auxPosServo=-90;
                ServoDriver();
                if(sAuxiliares.timeEco >= 700 && sAuxiliares.timeEco <= 1200){
                    MotorDriver();
                }else{
                    MotorDriver();
                }
                break;
    }

}
