#include <SPI.h>
#include <RF24.h>

#include <nRF24L01.h>
#include <SD.h>
#include <printf.h>
#include "Joystick.h"
#include "Timer.h"

#include "globalVariable.h"
#include "MPU6050_Interface.h"
#include "NRF24L01_Interface.h"
#include "CardSD.h"

#define USE_SD_CARD
#define USE_MPU6050
#define USE_NRF24L01


TransmissionPacket transmissionPacket;




//variabili globali: Seriale
Stream* SystemDebugPort;
boolean Command = false;
boolean dumpJoystickData = false;
char SerialPortBuffer[SerialBufferLenght];
uint8_t SerialBuff_Index = 0;


//variabili globali: ADC;
volatile uint8_t ADC_state = 0;


//variabili globali: Altro
const uint8_t SW_PIN[Jnumber] = {48, 46, 44, 42, 40, 38};
volatile Joystick* joysticks[Jnumber];
volatile uint8_t LED_errorState = 0;
volatile uint8_t activityState = 0;

volatile boolean last_SD_State = false;
volatile boolean NRF_error_led_state = false;
volatile boolean SD_error_led_state = false;
volatile boolean MPU_error_led_state = false;



//oggetti
SD_Card SDcard(SD_CSN);
MPU6050_Interface mpu6050_interface(&SDcard);
NRF24L01_Interface nrf24l01_interface(NRF_CE, NRF_CSN, &SDcard);

Timer timer(100);
Timer TransmissionTimer(10);
Timer NRF_error_led_timer(500);
Timer SD_error_led_timer(500);
Timer MPU_error_led_timer(500);
Timer activity(1000);
Timer SD_Test_Timer(3000);



inline __attribute__((always_inline)) void Serial_Loop();          
inline __attribute__((always_inline)) void RF_Loop();
inline __attribute__((always_inline)) void initilizeControls();
inline __attribute__((always_inline)) boolean initilizeSDcard();
inline __attribute__((always_inline)) boolean initializeNRF24L01();
inline __attribute__((always_inline)) boolean initializeMPU6050();


void setup() 
{
  //noInterrupts();           // Disabilita tutti gli interrupt

  boolean result;
  
  delay(500);
  DebugPort.begin(115200);
  printf_begin();

  if(DebugPort) {
    GlobalVariable::debugportAvailable = true;
  }


  //[=============================================== {Inizializzazione pin} ===============================================]//
  
  pinMode(NRF_ErrorLED, OUTPUT);
  pinMode(SD_ErrorLED,  OUTPUT);
  pinMode(MPU_ErrorLED, OUTPUT);
  pinMode(ACTIVITY_LED, OUTPUT);

  fastDigitalWrite(NRF_ErrorLED, LOW);
  fastDigitalWrite(SD_ErrorLED, LOW);
  fastDigitalWrite(MPU_ErrorLED, LOW);
  fastDigitalWrite(ACTIVITY_LED, LOW);

  //[=============================================== Inizializzazione componenti} ===============================================]//
  
  initilizeSDcard();
  initializeMPU6050();
  initializeNRF24L01();
  initilizeControls();
  
  // =================================== ADC =============================================== //
  
  ADC_state  = 0;     //stato iniziale
  ADMUX  = B01000000; //seleziono A0
  ADCSRB = B00000000; // Analog Input bank 1
  ADCSRA = B11011111; // ADC enable, start conversion, manual trigger mode, ADC interrupt enable, prescaler = 128

  if(GlobalVariable::debugportAvailable) {
    DebugPort.println(F("ADC configuration: "));
    DebugPort.print(F("*ADCSRB: "));DebugPort.println(ADCSRB, HEX);
    DebugPort.print(F("*ADCSRA: "));DebugPort.println(ADCSRA, HEX);
  }

  // =================================== TIMER5 =============================================== //
  TCCR5A = 0;               // Resetta il registro di controllo A
  TCCR5B = 0;               // Resetta il registro di controllo B
  TCNT5 = 0;                // Inizializza il contatore del timer 5 a 0

  // Il prescaler è 64, quindi il timer verrà eseguito a 16 MHz / 64 = 250 kHz
  // Il periodo del timer sarà quindi 1 / 250 kHz = 4 µs
  // Per ottenere 100 ms, è necessario contare 100 ms / 4 µs = 25000 conteggi
  // Poiché il timer è a 16 bit, utilizzeremo il valore 65535 - 25000 = 40535
  // per ottenere l'intervallo desiderato di 100 ms.
  OCR5A = 40535;

  
  TIMSK5 |= (1 << OCIE5A);              // Abilita l'interrupt di confronto A del timer 5
  TCCR5B |= (1 << CS51) | (1 << CS50);  // CS51 e CS50 abilitano il prescaler a 64


  last_SD_State = SDcard.is_SD_Connected();
  //interrupts();              // Abilita gli interrupt globali
}


void loop()  
{
  
  
  
  //======================= verifica stato SD =======================//
  if(SD_Test_Timer.Test()) 
  {
      //se ho collegato la scheda SD
      if(last_SD_State == false) {
          last_SD_State = initilizeSDcard();

          //se l'operazione è andata a buon fine
          if(last_SD_State) {
            initializeNRF24L01();
          }
      }
      else if(last_SD_State == true && !SDcard.is_SD_Connected()) {
          last_SD_State = false;
          SDcard.end();
      }
  }

  //======================= verifica stato NRF =======================//
  if(nrf24l01_interface.isDeviceAvailable() && TransmissionTimer.Test()) {
    RF_Loop(); 
  }
  

  
  
  
  if(GlobalVariable::debugportAvailable)
    Serial_Loop();

  /*if(packetLost && timerLedError.Test()) {
    LED_errorState = ~LED_errorState;
    digitalWrite(NRF_ErrorLED,LED_errorState);  
  }*/
}


// Questa funzione verrà chiamata ogni 0.100 secondi
ISR(TIMER5_COMPA_vect) {
  
  //LED di attività
  if(activity.Test()) {
    activityState = !activityState;
    digitalWrite(ACTIVITY_LED,activityState);
  }

  //LED di errore per l'SD
  if(!last_SD_State && SD_error_led_timer.Test()) {
    SD_error_led_state = !SD_error_led_state;
    fastDigitalWrite(SD_ErrorLED, SD_error_led_state);
  }
  else if(last_SD_State && SD_error_led_state) {
    SD_error_led_state = false;
    fastDigitalWrite(SD_ErrorLED, LOW);
  }

  //LED di errore per l'NRF
  if(nrf24l01_interface.isDeviceAvailable() && NRF_error_led_state) {
    NRF_error_led_state = false;
    fastDigitalWrite(NRF_ErrorLED, LOW);
  }
  else if(!nrf24l01_interface.isDeviceAvailable() && NRF_error_led_timer.Test()) {
    NRF_error_led_state = !NRF_error_led_state;
    fastDigitalWrite(NRF_ErrorLED, NRF_error_led_state);
  }

  //LED di errore per l'MPU6050
  if(!mpu6050_interface.available() && MPU_error_led_timer.Test()) {
    MPU_error_led_state = !MPU_error_led_state;
    fastDigitalWrite(MPU_ErrorLED, MPU_error_led_state);
  }
  else if(mpu6050_interface.available() && MPU_error_led_state) {
    MPU_error_led_state = false;
    fastDigitalWrite(MPU_ErrorLED, LOW);
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


inline boolean initilizeSDcard() 
{
  boolean result = false;

  #ifdef USE_SD_CARD
    if(GlobalVariable::debugportAvailable) {
      DebugPort.println(F("================================[ Inizializzazione SD-card ]================================"));
    }
    
    result = SDcard.init();

    if(GlobalVariable::debugportAvailable) {
      if(result)
        SDcard.dumpSDInformation();
      else
        SDcard.dumpError();
    }
  #endif
  return result;
}

inline boolean initializeNRF24L01() {
  if(GlobalVariable::debugportAvailable) {
    DebugPort.println();
    DebugPort.println(F("================================[ Inizializzazione NRF24L01 ]================================"));
  }

  nrf24l01_interface.reset();
  boolean res = nrf24l01_interface.init();

  if(GlobalVariable::debugportAvailable) {
    nrf24l01_interface.printDetails();
    DebugPort.println(F("---------------------------------------------------------------------------------------------"));
  }
  return res;
}


inline boolean initializeMPU6050() 
{
  boolean result = false;

  if(GlobalVariable::debugportAvailable) {
    DebugPort.println();
    DebugPort.println(F("================================[ Inizializzazione MPU6050 ]================================"));
  }

  result = mpu6050_interface.init();
   

  if(GlobalVariable::debugportAvailable) {
    if(!result)
      DebugPort.println(F("MPU6050 initialization failed"));
    else
      DebugPort.println(F("MPU6050 initialization success"));
    DebugPort.println(F("---------------------------------------------------------------------------------------------"));
  }
 
  return result;
}

inline void initilizeControls() {
  if(GlobalVariable::debugportAvailable) {
    DebugPort.println();
    DebugPort.println(F("================================[ Inizializzazione I/O ]================================"));
  }

  for(uint8_t i = 0; i < Jnumber; i++) {
    joysticks[i] = new Joystick(SW_PIN[i], A0 + i*2, A1 + i*2, Bt_FALLING);
  }

  if(GlobalVariable::debugportAvailable) {
    DebugPort.println(F("Inizializzazione Joystick Completata"));
  }
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
    
  
    //invio i dati ai dispositivi
    for(uint8_t i = 0; i < 1; i++) {
      //radio.openWritingPipe(WritingPipe[i]);
      nrf24l01_interface.write(&transmissionPacket, sizeof(TransmissionPacket));
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
      nrf24l01_interface.printDetails();
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
