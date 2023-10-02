#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#include "MyUtility.h"

//#define DEBUG
#define AS_TRANSMITTER
//#define AS_RECEIVER

#if defined(AS_TRANSMITTER) && defined(AS_RECEIVER)
  #error Can't be both transmitter and receiver
#endif
#if !defined(AS_TRANSMITTER) && !defined(AS_RECEIVER)
  #error Must be a transmitter or a receiver
#endif

#define NODE0_ADDRESS "00001"
#define NODE1_ADDRESS "00002"
#define CHANNEL 0x60

#ifdef AS_TRANSMITTER
  #define NRF_IRQ_PIN 2
  #define STATUS_LED_PIN 3
  #define LEFT_BUTTON_PIN 4
  #define RIGHT_BUTTON_PIN 5
  #define FORWARD_BUTTON_PIN 6 
  #define BACKWORD_BUTTON_PIN 7
  #define CSN_PIN 10
  #define CE_PIN 8
#else
  #define STATUS_LED_PIN 3
  #define LEFT_RELAY_PIN 4
  #define RIGHT_RELAY_PIN 5
  #define FORWARD_RELAY_PIN 6 
  #define BACKWORD_RELAY_PIN 7
  #define CSN_PIN 10
  #define CE_PIN 8
  #define CONNECTION_LOST_LED_PIN 9
#endif

#define NO_BLINK_LED_SEQUENZE 0
#define NRF_ERROR_LED_SEQUENZE 1
#define NRF_CONNECTION_LOST_LED_SEQUENZE 2
#define NO_ERROR_AND_CONNECTED_LED_SEQUENZE 3



typedef struct payloadPacket 
{
  uint8_t sw1;
  uint8_t sw2;
  uint8_t sw3;
  uint8_t sw4;
  //uint8_t status;

} Packet;


#ifdef AS_TRANSMITTER
  const uint8_t READING_PIPE[] = {NODE1_ADDRESS}; //leggo dal canale del nodo 1
  const uint8_t WRITING_PIPE[] = {NODE0_ADDRESS}; //scrivo sul mio canale

  unsigned long lastPacket_sended_time = millis();

  const uint8_t pins[][2] PROGMEM = {
    {LEFT_BUTTON_PIN, INPUT}, 
    {RIGHT_BUTTON_PIN, INPUT},
    {FORWARD_BUTTON_PIN, INPUT},
    {BACKWORD_BUTTON_PIN, INPUT},
    {STATUS_LED_PIN, OUTPUT},
    {NRF_IRQ_PIN, INPUT}
  };

#else
  const uint8_t READING_PIPE[] = {NODE0_ADDRESS}; //leggo dal canale del nodo 0
  const uint8_t WRITING_PIPE[] = {NODE1_ADDRESS}; //scrivo sul mio canale

  const uint8_t pins[][2] PROGMEM = {
    {LEFT_RELAY_PIN, INPUT}, 
    {RIGHT_RELAY_PIN, INPUT},
    {FORWARD_RELAY_PIN, INPUT},
    {BACKWORD_RELAY_PIN, INPUT},
    {STATUS_LED_PIN, OUTPUT},
    {CONNECTION_LOST_LED_PIN, OUTPUT}
  };
#endif


uint8_t NRF_WrError_sequenze[]  = {1, 1, 0, 0, 1, 1, 0, 0};
uint8_t NRF_error_sequenze[]    = {1, 1, 0, 0, 1, 1, 0, 0, 0, 0};
uint8_t noError_sequenze[]      = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t noBlink_sequenze[]      = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


typedef struct Animation_ {
  uint8_t stepIndex;
  uint8_t stepCount;
  uint8_t* LED_animation_Array;
}
LED_Animation;


struct LED_Animations_  
{
  uint8_t selectedAnimation = 0;
  uint8_t led_state = 0;
  
  LED_Animation animations[4] = {
    {0, sizeof(noBlink_sequenze)/sizeof(noBlink_sequenze[0])        , &noBlink_sequenze[0]},
    {0, sizeof(NRF_error_sequenze)/sizeof(NRF_error_sequenze[0])    , &NRF_error_sequenze[0]},
    {0, sizeof(NRF_WrError_sequenze)/sizeof(NRF_WrError_sequenze[0]), &NRF_WrError_sequenze[0]},
    {0, sizeof(noError_sequenze)/sizeof(noError_sequenze[0])        , &noError_sequenze[0]},
  };

} LED_Animations;



boolean dataAvailable = false;
boolean serialIsAvailable = false;
long timerLED = 0;
long timerPacket = 0;
RF24 radio(CE_PIN, CSN_PIN);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
Packet packet;


inline __attribute__((always_inline)) void RX_loop();
inline __attribute__((always_inline)) void TX_loop();


void setup() 
{
  noInterrupts();           // Disabilita tutti gli interrupt
  setLED_Animation(NO_BLINK_LED_SEQUENZE);

  SPI.begin();
  Serial.begin(115200);
  serialIsAvailable = Serial ? true : false;

  if(serialIsAvailable) {
    #ifdef AS_TRANSMITTER
      Serial.println(F("working as transmitter"));
    #else
      Serial.println(F("working as receiver"));
    #endif
  }
  
  for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) 
  {
    pinMode(pins[i][0], pins[i][1]);
    if(pins[i][1] == OUTPUT) {
      digitalWrite(pins[i][0], LOW);
    }
  }

  

  //==================================================================//
  //impostazione TIMER1
  TCCR1A = 0;               // Resetta il registro di controllo A
  TCCR1B = 0;               // Resetta il registro di controllo B
  TCNT1 = 0;                // Inizializza il contatore del timer 1 a 0
  
  // Imposta il prescaler per ottenere il periodo desiderato
  // Il prescaler è 64, quindi il timer verrà eseguito a 16 MHz / 64 = 250 kHz
  // Il periodo del timer sarà quindi 1 / 250 kHz = 4 µs
  // Per ottenere 100 ms, è necessario contare 100 ms / 4 µs = 25000 conteggi
  // Poiché il timer è a 16 bit, utilizzeremo il valore 65535 - 25000 = 40535
  // per ottenere l'intervallo desiderato di 100 ms.
  OCR1A = 40535;
  
  // Abilita l'interrupt di confronto A del timer 1
  TIMSK1 |= (1 << OCIE1A);
  
  // Abilita il timer 1 con il prescaler 64
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // WGM12 abilita il timer CTC mode, CS11 e CS10 abilitano il prescaler a 64

  interrupts();              // Abilita gli interrupt globali
  //==================================================================//


  //==================================================================//
  //inizializzazione NRF24L01

  if (!radio.begin()) {
    setLED_Animation(NRF_ERROR_LED_SEQUENZE);
    do {
      if(serialIsAvailable) 
        Serial.println(F("radio hardware is not responding!!"));
      
      delay(1000);

      if(serialIsAvailable) 
        Serial.println(F("new attempt"));
    }
    while(!radio.begin());
    setLED_Animation(NO_BLINK_LED_SEQUENZE);
  }

  if(serialIsAvailable) {
    Serial.println(F("radio hardware detected"));
    Serial.print(F("READING_PIPE: "));
    Serial.println((long)READING_PIPE, HEX);
    Serial.print(F("WRITING_PIPE: "));
    Serial.println((long)WRITING_PIPE, HEX);
  }
  
  radio.setPALevel(RF24_PA_MAX);              // RF24_PA_MAX is default.
  radio.setChannel(CHANNEL);                  // set the RF channel 
  radio.setPayloadSize(sizeof(Packet));       // size of the payload 
  radio.setAutoAck(true);                     // enable auto ack
  radio.enableAckPayload();                   // enable ack payload
  radio.setDataRate(RF24_1MBPS);              // set the data rate
  radio.setRetries(0, 15);                    // set the retry counts
  radio.openWritingPipe(WRITING_PIPE);        // always uses pipe 0
  radio.openReadingPipe(1, READING_PIPE);     // using pipe != 0

  #ifdef AS_TRANSMITTER
    attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), NRF_transmitIRQ, RISING);
    radio.maskIRQ(1,0,1);                     // data sent interrupt and data failed interrupt
    radio.stopListening();                    // put radio in TX mode 
  #else
    radio.startListening();   // put radio in RX mode
    //radio.maskIRQ(1,1,0);                     // data ready interrupt
    //attachInterrupt(digitalPinToInterrupt(2), getPacketIRQ, RISING);
  #endif

  
  if(serialIsAvailable) {
    printf_begin();             // needed only once for printing details
    radio.printPrettyDetails(); // (larger) function that prints human readable data
  } 
  //==================================================================//
  setLED_Animation(NO_BLINK_LED_SEQUENZE);
}

void loop() 
{
  #ifdef AS_TRANSMITTER
    if(millis() - timerPacket >=  40) {
      timerPacket = millis();
      TX_loop();
    }
  #else
    RX_loop();  
  #endif
}


ISR(TIMER1_COMPA_vect) 
{
  LED_Animation *pAnimation = &LED_Animations.animations[LED_Animations.selectedAnimation];

  //verifico se devo cambiare il valore del LED
  if(pAnimation->LED_animation_Array[pAnimation->stepIndex] != LED_Animations.led_state) {
    digitalWrite(STATUS_LED_PIN, !LED_Animations.led_state);
    LED_Animations.led_state = pAnimation->LED_animation_Array[pAnimation->stepIndex];
  }

  //incremento l'indice dell'animazione
  pAnimation->stepIndex = (pAnimation->stepIndex + 1) % pAnimation->stepCount;
}

#ifdef AS_TRANSMITTER
void NRF_transmitIRQ() {
  //se è passato 1s dall'ultimo pacchetto ricevuto
  if(millis() - lastPacket_sended_time  >= 1000) {
    setLED_Animation(NRF_CONNECTION_LOST_LED_SEQUENZE);
  }
}
#endif


void getPacketIRQ() {
  dataAvailable = true;
}

inline __attribute__((always_inline))
void setLED_Animation(uint8_t index) {
  if(index > sizeof(LED_Animations.animations)/sizeof(LED_Animations.animations[0])) {
    if(serialIsAvailable)
      Serial.println(F("Animation index out of range"));    
    return;
  }

  LED_Animations.selectedAnimation = index;
  
  if(serialIsAvailable) {
    Serial.print(F("Animation index: "));
    Serial.println(index);
  }
}

#ifdef AS_RECEIVER
inline __attribute__((always_inline))
void RX_loop() {
  uint8_t pipe;
    dataAvailable = false;
    // is there a payload? get the pipe number that recieved it
    if (radio.available(&pipe)) 
    {              
      timerPacket = millis(); //mi salvo il momento in cui ho ricevuto l'ultimo pacchetto
      uint8_t *p = (uint8_t *) &packet;
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&packet, bytes);             // fetch payload from FIFO

      for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) {
        //digitalWrite(pins[i], p[i]);
      }
      //digitalWrite(STATUS_LED_PIN, LOW);

      #ifdef DEBUG
        Serial.print(F("Received "));
        Serial.print(bytes);  // print the size of the payload
        Serial.print(F(" bytes on pipe "));
        Serial.print(pipe);  // print the pipe number
        Serial.print(F(": "));
      
        for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) {
          Serial.print(p[i]);
          Serial.print(F(" "));
        }
        Serial.println();
      #endif
    }
    //se è passato un secondo dall'umtimo pacchetto
    //spengo tutto e attivo il LED di errore
    else if(millis() - timerPacket >= 1000 )
    {
      for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) {
        //digitalWrite(pins[i], LOW);
      }
      //digitalWrite(STATUS_LED_PIN, HIGH);
      delay(100);
    }
}
#endif

#ifdef AS_TRANSMITTER
inline __attribute__((always_inline))
void TX_loop() 
{
  static uint8_t *p = (uint8_t *) &packet;

  //leggo i valori dei pulsanti e li salvo in packet
  *(p + 0) = digitalRead(FORWARD_BUTTON_PIN);
  *(p + 1) = digitalRead(BACKWORD_BUTTON_PIN);
  *(p + 2) = digitalRead(LEFT_BUTTON_PIN);
  *(p + 3) = digitalRead(RIGHT_BUTTON_PIN);

  //invio i dati e cronometro il tempo
  unsigned long start_timer = micros();                // start the timer
  bool report = radio.write(&packet, sizeof(packet));  // transmit & save the report
  unsigned long end_timer = micros();                  // stop the timer

  
  //resetto il contatore 
  if(report) {
    lastPacket_sended_time = millis();
    
    if(LED_Animations.selectedAnimation != NO_ERROR_AND_CONNECTED_LED_SEQUENZE) {
      setLED_Animation(NO_ERROR_AND_CONNECTED_LED_SEQUENZE);
    }
  }
    
  if(serialIsAvailable) {
    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);        // print the timer result
      Serial.print(F(" us. Sent: "));

      for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) {
        Serial.print(p[i]);
        Serial.print(F(" "));
      }
      Serial.println();
    }
    else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);        // print the timer result
      Serial.print(F(" us."));
      Serial.println();
    }
  }
}
#endif