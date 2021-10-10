#include "mbed.h"
#include <stdbool.h>

#define NUMBUTT             4
#define MAXLED              4
#define NUMBEAT             4
#define HEARBEATINTERVAL    100

//a ver si compila
typedef enum{ //contiene estados de la MEF //boton como pullup
    BUTTON_DOWN,    //0
    BUTTON_UP,      //1
    BUTTON_FALLING, //2
    BUTTON_RISING   //3
}_eButtonState;

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
}_sTeclas;

_sTeclas ourButton[NUMBUTT];
//                0001 ,  0010,  0100,  1000


uint16_t mask[]={0x0001,0x0002,0x0004,0x0008};

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
        OTHERS
    }_eID;

typedef struct{              //Estructura de datos para el puerto serie
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t cheksumtx;       //!< Cheksumm Tx
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
    uint8_t payload[32];     //!< Buffer para el Payload de datos recibidos
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

_eButtonState myButton;//asignar estados a la MEF


void startMef(uint8_t indice);
void actuallizaMef(uint8_t indice );
void togleLed(uint8_t indice);
void manejadorLed(uint8_t mask);
void onDataRx(void);
void decodeProtocol(void);
void decodeData(void);
void sendData(void);
void OnTimeOut(void);
void hearbeatTask(void);
void ServoDriver();
//void myTrigger();
//void startEcho();


BusIn buttonArray(PB_6,PB_7,PB_8,PB_9);
BusOut leds(PB_12,PB_13,PB_14,PB_15);

PwmOut Servo(PB_10);

DigitalOut HEARBEAT(PC_13); //!< Defino la salida del led
//Serial
Serial pcCom(PA_9,PA_10,115200); //!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT
Timer miTimer; //!< Timer general

Ticker timerGeneral;
int8_t auxPosServo=0 ;//auxiliar global posicion del servo

int main()
{
    int hearbeatTime=0;
    miTimer.start();
    myFlags.individualFlags.checkButtons=false;
    hearBeatEvent=0;
    pcCom.attach(&onDataRx,Serial::RxIrq);
    timerGeneral.attach_us(&OnTimeOut, 50000);

    //hola
    for(uint8_t indice=0; indice<NUMBUTT;indice++){
        startMef(indice);
    }

    while(true)
    {
        if(myFlags.individualFlags.checkButtons){
            myFlags.individualFlags.checkButtons=false;
            for(uint8_t indice=0; indice < NUMBUTT; indice++){
                if (buttonArray & mask[indice]){
                    ourButton[indice].event =EV_NOT_PRESSED;
                }else{
                    ourButton[indice].event =EV_PRESSED; 
                }
                actuallizaMef(indice);
            }
        }
        if ((miTimer.read_ms()-hearbeatTime)>=HEARBEATINTERVAL){
            hearbeatTime=miTimer.read_ms();
            hearbeatTask();
        }
        if(datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx) 
            decodeProtocol();

        if(datosComProtocol.indexReadTx!=datosComProtocol.indexWriteTx) 
            sendData();

    }
    return 0;
}

/*void myTrugger(){
    TriggerPin = 0;
}

void startEcho(){
    start.echo
}
*/


void startMef(uint8_t indice){//MEF para DEBOUNCE de botones
   ourButton[indice].estado=BUTTON_UP;
}


void actuallizaMef(uint8_t indice){

    switch (ourButton[indice].estado)
    {
    case BUTTON_DOWN:
        if(ourButton[indice].event )
           ourButton[indice].estado=BUTTON_RISING;
    
    break;
    case BUTTON_UP:
        if(!(ourButton[indice].event))
            ourButton[indice].estado=BUTTON_FALLING;
    
    break;
    case BUTTON_FALLING:
        if(!(ourButton[indice].event))
        {
            ourButton[indice].timeDown=miTimer.read_ms();
            ourButton[indice].estado=BUTTON_DOWN;
            //Flanco de bajada
        }
        else
            ourButton[indice].estado=BUTTON_UP;    

    break;
    case BUTTON_RISING:
        if(ourButton[indice].event){
            ourButton[indice].estado=BUTTON_UP;
            //Flanco de Subida
            ourButton[indice].timeDiff=miTimer.read_ms()-ourButton[indice].timeDown;
            togleLed(indice);
        }

        else
            ourButton[indice].estado=BUTTON_DOWN;
    
    break;
    
    default:
    startMef(indice);
        break;
    }
}

/************  Funciónes para manejar los LEDS ***********************/

void togleLed(uint8_t indice){
    uint16_t ledsAux=leds, auxmask=0;
    auxmask |= 1<<indice;
    if(auxmask & leds)
        ledsAux &= ~(1 << (indice)) ; 
    else
         ledsAux |= 1 << (indice) ;
    leds = ledsAux ;
}


void manejadorLed(uint8_t mask){
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
}

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
            myWord.ui16[0]=leds;
            auxBuffTx[indiceAux++]=myWord.ui8[0];
            auxBuffTx[indiceAux++]=myWord.ui8[1];
            auxBuffTx[NBYTES]=0x04;
            break;

        case SET_LEDS:
            auxBuffTx[indiceAux++]=SET_LEDS;
            myWord.ui8[0]=datosComProtocol.payload[2];
            myWord.ui8[1]=datosComProtocol.payload[3];
            auxBuffTx[NBYTES]=0x02;
            manejadorLed(myWord.ui16[0]);
            break;

        case SERVO_ANG:
            auxBuffTx[indiceAux++]=SERVO_ANG;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            auxPosServo = datosComProtocol.payload[2];
            ServoDriver();
            break;

        case 4: //cambiar por el nombre del comando que se quiera agregar
            break;
        case 5://cambiar por el nombre del comando que se quiera agregar
            break;

        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
   for(uint8_t a=0 ;a < indiceAux ;a++)
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

void hearbeatTask(void) //Función para hacer el hearbeats
{
    if(hearBeatEvent < NUMBEAT){
        HEARBEAT=!HEARBEAT;
        hearBeatEvent++;
    }else{
        HEARBEAT=1;
        hearBeatEvent = (hearBeatEvent>=25) ? (0) : (hearBeatEvent+1);    
    }
}

/* Servicio de Interrupciones*/

void onDataRx(void)
{
    while (pcCom.readable())
    {
        datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
    }
}

void OnTimeOut(void)
{
    if(!myFlags.individualFlags.checkButtons)
        myFlags.individualFlags.checkButtons=true;
}

void ServoDriver(void)
{
    Servo.period_ms(20);
    Servo.pulsewidth_us(1500+((auxPosServo*1000)/180));
}