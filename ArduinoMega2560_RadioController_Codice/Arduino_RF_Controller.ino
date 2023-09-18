#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>
#include "Joystick.h"
#include "Timer.h"


#define DebugPort Serial 
#define Jnumber  6
#define SerialBufferLenght 32
#define CE 20
#define CSN 21
#define ErrorLED 12

//variabili globali: Seriale
boolean DebugPortAvailable = false;
boolean Command = false;
boolean dumpJoystickData = false;
char SerialPortBuffer[SerialBufferLenght];
uint8_t SerialBuff_Index = 0;


//variabili globali: ADC;
volatile uint8_t ADC_state = 0;


//variabili globali: nRF24l01
boolean radioDeviceAvailable = false;
boolean packetLost = true;
unsigned long transmissionStart;
const uint8_t comunicationChannel = 100;
const uint64_t ReadingPipe[2] = {0xE6AA00000010, 0xE6AA00000020}; //slaveAddress reading
const uint64_t WritingPipe[2] = {0xE6AA00000011, 0xE6AA00000022}; //slaveAddress writing


//variabili globali: Altro
const uint8_t SW_PIN[Jnumber] = {48, 46, 44, 42, 40, 38};
volatile Joystick* joysticks[Jnumber];
uint8_t LED_errorState = 0;
uint8_t activityState = 0;


//oggetti
RF24 radio(CE,CSN); 
Timer timer(100);
Timer TransmissionTimer(10);
Timer timerLedError(250);
Timer activity(250);

//strutture
struct TransmissionPacket
{
  int16_t J1x;
  int16_t J2x;
  int16_t J3x;
  int16_t J4x;
  int16_t J5x;
  int16_t J6x;
  
  int16_t J1y;
  int16_t J2y;
  int16_t J3y;
  int16_t J4y;
  int16_t J5y;
  int16_t J6y;
  
  uint8_t  sw1;
  uint8_t  sw2;
  uint8_t  sw3;
  uint8_t  sw4;
  uint8_t  sw5;
  uint8_t  sw6;

  uint32_t Rx;
  uint32_t RY;
  uint32_t RZ;
  uint32_t Tm;
  
}transmissionPacket;

//massimo 32 Byte
struct PayloadStruct {
  unsigned long nodeID;
  unsigned long payloadID;
};
PayloadStruct payload;




inline void Serial_Loop() __attribute__((always_inline));
inline void RF_Loop() __attribute__((always_inline));


void setup() 
{
  delay(500);
  
  DebugPort.begin(115200);
  printf_begin();
  
  if(DebugPort) DebugPortAvailable = true;

  if(DebugPortAvailable)DebugPort.println(F("================ Inizializzazione Controller ================"));
// =================================== Joystick Switches pot. LED =============================================== //
  
  if(DebugPortAvailable)DebugPort.println(F("Inizializzazione Dispositivi I/O: "));

  for(uint8_t i = 0; i < Jnumber; i++) {
    joysticks[i] = new Joystick(SW_PIN[i], A0 + i*2, A1 + i*2, Bt_FALLING);
    if(DebugPortAvailable){
      DebugPort.print("creazione j");
      DebugPort.println(i + 1);
    }
  }

  pinMode(ErrorLED,OUTPUT);
  pinMode(11,OUTPUT);
  digitalWrite(ErrorLED,LOW);
  
  if(DebugPortAvailable) {
    DebugPort.println(F("Completato"));
    DebugPort.println(F("Inizializzazione ADC: "));
  }
// =================================== ADC =============================================== //
  
  ADC_state  = 0;     //stato iniziale
  ADMUX  = B01000000; //seleziono A0
  ADCSRB = B00000000; // Analog Input bank 1
  ADCSRA = B11011111; // ADC enable, start conversion ,manual trigger mode, ADC interrupt enable, prescaler = 128

  if(DebugPortAvailable) {
    DebugPort.print(F("ADCSRB: "));DebugPort.println(ADCSRB, HEX);
    DebugPort.print(F("ADCSRA: "));DebugPort.println(ADCSRA, HEX);
  }
 
  if(DebugPortAvailable)DebugPort.println(F("Completato"));
  if(DebugPortAvailable)DebugPort.println(F("Inizializzazione nRF24L01: "));
// =================================== nRF24L01 =============================================== //

  if (!radio.begin()) {
    if(DebugPortAvailable)
      DebugPort.println(F("Il dispositivo non risponde\nInizialiizaione fallita"));
  }
  else 
  {
    radioDeviceAvailable = true;
    radio.setChannel(comunicationChannel);    //canale radio
    radio.setAutoAck(false);                   //pacchetto di ricezione
    //radio.enableDynamicPayloads();            // ?
    //radio.setPayloadSize(sizeof(payload));
    radio.setRetries(0,15);                   //tempo tra ritentativi, numero di tentativi
    radio.setDataRate(RF24_250KBPS);          //velocità di trasmissione
    //radio.openReadingPipe(0,ReadingPipe[0]);  //device 1, canale di lettura 0
    //radio.openReadingPipe(1,ReadingPipe[1]);  //device 2, canale di lettura 1

     //se collego la USB alla scheda
    if(DebugPortAvailable) 
      radio.setPALevel(RF24_PA_LOW);  //potenza minima
    else
      radio.setPALevel(RF24_PA_HIGH); //potenza massima

    radio.openWritingPipe(WritingPipe[0]);
    radio.powerUp();
    //radio.startListening();
    radio.stopListening();
      
    if(DebugPortAvailable)DebugPort.println(F("Completata"));
  }
  delay(1500);
}

void loop() 
{
  if(TransmissionTimer.Test())
    RF_Loop();
  
  if(DebugPortAvailable)
    Serial_Loop();

  if(packetLost && timerLedError.Test()) {
    LED_errorState = ~LED_errorState;
    digitalWrite(ErrorLED,LED_errorState);  
  }
}


// Interrupt service routine for ADC conversion complete
ISR(ADC_vect) 
{
  //valori dell'ultima conversione
  unsigned char adcl = ADCL;
  unsigned char adch = ADCH;
  unsigned int adcVal = (adch << 8) | adcl;
  uint8_t index = ADC_state/2;


  //dove salvo il risultato della conversione
  //se sto leggendo i joystick
  if(ADC_state < Jnumber*2) 
  {
    if(ADC_state % 2 == 0) 
      joysticks[index]->ValueX = (index % 2 == 1) ? adcVal : (1023 - adcVal);   //map(adcVal, 0, 1023, -512, +511) * ((ADC_state / 2) % 2 == 1) ? 1 : -1;
    else
      joysticks[index]->ValueY = (index % 2 == 1) ? (1023 - adcVal) : adcVal;  //map(adcVal, 0, 1023, -512, +511) * ((ADC_state / 2) % 2 == 1) ? -1 : 1;
    
    ADC_state++;
    //(joysticks[ADC_state/2]->getPinY() - 54    A0 - 54 == 0
    
  }
  else {
    ADC_state = 0;
  }

  //imposto il canale
  ADMUX  = B01000000 | (ADC_state & 0b0000111);  //MUX[0:2] channel, MUX[3] Gain e MUX[4] Differential Input  
  ADCSRB = B01000000 | (ADC_state & 0b0001000);  //MUX[5] channel   => A0:A7 con MUX[0:2] e A8:A15 con MUX[5]
  ADCSRA = B11011111; // ADC enable, start conversion ,manual trigger mode, ADC interrupt enable, prescaler = 128
}


void makePacket()
{
   //puntatori locazioni
   int16_t *pjx = &transmissionPacket.J1x;
   int16_t *pjy = &transmissionPacket.J1y;
   uint8_t *psw = &transmissionPacket.sw1;
  
   for(uint8_t i = 0; i < Jnumber; i++) 
   {   
      //copio i contenuti e incremento di 2 Byte(int16_t) i puntatori
      *pjx++ = map(joysticks[i]->ValueX, 0, 1023, -512, 512) * ((Jnumber % 2 == 0) ? (+1) : (-1)); 
      *pjy++ = map(joysticks[i]->ValueY, 0, 1023, -512, 512) * ((Jnumber % 2 == 0) ? (-1) : (+1));
      
      // moltiplico per -1 perchè i joystick displari sono specchiati //
  
      //copio il contenuto e incremento di 1 Byte(int16_t) il puntatore
      *psw++ = joysticks[i]->ButtonFunctionAvailable();  
   }
}


void RF_Loop()
{
    //radio.stopListening();
    makePacket();
    if(activity.Test()) {
      activityState = !activityState;
      digitalWrite(11,activityState);
    }
    
  
    //invio i dati ai dispositivi
    for(uint8_t i = 0; i < 1; i++) {
      //radio.openWritingPipe(WritingPipe[i]);
      radio.write(&transmissionPacket, sizeof(TransmissionPacket));
   }

   //radio.startListening();
}


void Serial_Loop()
{
  if(DebugPort.available()) 
  {
    char ch = DebugPort.read();
    if(ch == '\n' || ch == '\r' || SerialBuff_Index == SerialBufferLenght - 1) {
      Command = true;
      DebugPort.print("Command: ");
      DebugPort.println(SerialPortBuffer);
    }
    else {
      SerialPortBuffer[SerialBuff_Index++] = (ch >= 97 && ch <= 122) ? (ch -= 32) : ch; //toUpper
    }
  }
  
  if(Command)
  {
    Command = false;
    String str;

    for(uint8_t i = 0; i < SerialBuff_Index; i++)
      str += SerialPortBuffer[i];
      
    SerialBuff_Index = 0; 
    
    if(str == "HELP") {
      //printCommands()
    }
    else if(str == "STOP") {
      dumpJoystickData = false;
    }
    else if(str == "JS") {
      dumpJoystickData = !dumpJoystickData;
    }
    else if(str == "RF") {
      radio.printDetails();
    }
    else if(str == "TP") 
    {
      //puntatori locazioni
      int16_t *pjx = &transmissionPacket.J1x;
      int16_t *pjy = &transmissionPacket.J1y;
      uint8_t *psw = &transmissionPacket.sw1;
      
      for(uint8_t i = 0; i < Jnumber; i++) 
      {   
        char buf[40];
        sprintf(buf, "J%01d [x: %03d, y: %03d, sw: %01d]", i + 1, *pjx++,  *pjy++, *psw++);
        DebugPort.println(buf);
      }
    }
  }


  if(dumpJoystickData && timer.Test()) {
    for(int i = 0; i < Jnumber; i++) 
    {
      char buf1[5];
      char buf2[5];
      
      DebugPort.print("J"); DebugPort.print(i);
      DebugPort.print("  [X = ");  
      //DebugPort.print(joysticks[i]->ValueX < 0 ? "-" : " ");  

      //sprintf(buf1, "%04d", joysticks[i]->ValueX < 0 ? (joysticks[i]->ValueX * -1) : joysticks[i]->ValueX);
      sprintf(buf1, "%04d", joysticks[i]->ValueX);
      DebugPort.print(buf1);
      
      DebugPort.print(", Y = "); 
      //DebugPort.print(joysticks[i]->ValueY < 0 ? "-" : " ");    

      //sprintf(buf2, "%04d", (joysticks[i]->ValueY < 0 ? (joysticks[i]->ValueY * -1) : joysticks[i]->ValueY));
      sprintf(buf2, "%04d", joysticks[i]->ValueY);
      DebugPort.print(buf2);
      
      DebugPort.print(", SW = "); DebugPort.print(joysticks[i]->getState());
      DebugPort.print("]");

      /*
      sprintf(buffer, "J1 [X = %d, Y = %d, SW = %d] | J2 [X = %d, Y = %d, SW = %d] | J3 [X = %d, Y = %d, SW = %d] | J4 [X = %d, Y = %d, SW = %d] | J5 [X = %d, Y = %d, SW = %d] | J6 [X = %d, Y = %d, SW = %d]", 
      joysticks[index]->ReadX(), joysticks[index]->ReadY(), joysticks[index++]->getState(),
      joysticks[index]->ReadX(), joysticks[index]->ReadY(), joysticks[index++]->getState(),
      joysticks[index]->ReadX(), joysticks[index]->ReadY(), joysticks[index++]->getState(),
      joysticks[index]->ReadX(), joysticks[index]->ReadY(), joysticks[index++]->getState(),
      joysticks[index]->ReadX(), joysticks[index]->ReadY(), joysticks[index++]->getState(),
      joysticks[index]->ReadX(), joysticks[index]->ReadY(), joysticks[index++]->getState());
      */

      if(i!= Jnumber -1)DebugPort.print(" | ");
    }
    DebugPort.println();
  }
}
