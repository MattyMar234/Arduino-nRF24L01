#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>


//#define DEBUG
//#define AS_TRANSMITTER
#define AS_RECEIVER
#define ENABLE_LIMIT_SW_IRQ

#if defined(AS_TRANSMITTER) && defined(AS_RECEIVER)
  #error Can't be both transmitter and receiver
#endif
#if !defined(AS_TRANSMITTER) && !defined(AS_RECEIVER)
  #error Must be a transmitter or a receiver
#endif



////////////////////////////////////////////////////////////////
//  Costanti
////////////////////////////////////////////////////////////////
#define NODE0_ADDRESS "00001"
#define NODE1_ADDRESS "00002"
#define CHANNEL 0x60

#ifdef AS_TRANSMITTER
  //LED
  #define STATUS_LED_PIN 3

  //BT
  #define LEFT_BUTTON_PIN 4
  #define RIGHT_BUTTON_PIN 5
  #define FORWARD_BUTTON_PIN 6 
  #define BACKWORD_BUTTON_PIN 7

  //NRF
  #define NRF_IRQ_PIN 2
  #define CSN_PIN 10
  #define CE_PIN 8
#else
  //LED
  #define CONNECTION_LOST_LED_PIN A3
  #define STATUS_LED_PIN A0

  //LIMIT_SW
  #define LEFT_LIMIT_SW A1
  #define RIGHT_LIMIT_SW A2
  #define LIMIT_SW_IRQ_PIN 3

  //RELAY
  #define LEFT_RELAY_PIN 4
  #define RIGHT_RELAY_PIN 5
  #define FORWARD_RELAY_PIN 6 
  #define BACKWORD_RELAY_PIN 7

  //NRF
  #define CSN_PIN 10
  #define CE_PIN 9
  #define NRF_IRQ_PIN 2
#endif

#define NO_BLINK_LED_SEQUENZE 0
#define NRF_ERROR_LED_SEQUENZE 1
#define NRF_CONNECTION_LOST_LED_SEQUENZE 2
#define NO_ERROR_AND_CONNECTED_LED_SEQUENZE 3


////////////////////////////////////////////////////////////////
//  Varibili
////////////////////////////////////////////////////////////////
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
    {NRF_IRQ_PIN, INPUT},
  };

  const uint8_t pins_mode[] PROGMEM = {
    INPUT, INPUT, INPUT, INPUT, OUTPUT, INPUT
  };

#else
  const uint8_t READING_PIPE[] = {NODE0_ADDRESS}; //leggo dal canale del nodo 0
  const uint8_t WRITING_PIPE[] = {NODE1_ADDRESS}; //scrivo sul mio canale

  unsigned long lastPacket_recived_time = millis();
  
  const uint8_t pins[][2] PROGMEM = {
    {LEFT_RELAY_PIN, OUTPUT},
    {RIGHT_RELAY_PIN, OUTPUT},
    {FORWARD_RELAY_PIN, OUTPUT},
    {BACKWORD_RELAY_PIN, OUTPUT},
    {STATUS_LED_PIN, OUTPUT},
    {CONNECTION_LOST_LED_PIN, OUTPUT},
    {NRF_IRQ_PIN, INPUT},
    {LEFT_LIMIT_SW, INPUT},
    {RIGHT_LIMIT_SW, INPUT}
  };
#endif

//funzione per far ripartire il codice da capo (software reset)
void(* resetFunc) (void) = 0;


const uint8_t NRF_WrError_sequenze[] = {1, 1, 0, 0, 1, 1, 0, 0};
const uint8_t NRF_error_sequenze[]   = {1, 0, 1, 0, 0, 0, 0};
const uint8_t noError_sequenze[]     = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t noBlink_sequenze[]     = {0, 0};

//Pacchetto dati
typedef struct payloadPacket {
  uint8_t sw1;
  uint8_t sw2;
  uint8_t sw3;
  uint8_t sw4;
} 
Packet;


typedef struct Animation_ {
  uint8_t stepIndex;
  uint8_t stepCount;
  const uint8_t* LED_animation_Array;
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
} 
LED_Animations;


RF24 radio(CE_PIN, CSN_PIN);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
Packet packet;

boolean IsSerialAvailable = false;
long timerPacket = 0;




void setup() 
{
  // Disabilita tutti gli interrupt
  noInterrupts();           
  setLED_Animation(NO_BLINK_LED_SEQUENZE);

  //////////////////////////////////////////////////////////////////////
  //inizilizzazione periferiche
  //////////////////////////////////////////////////////////////////////
  SPI.begin();
  Serial.begin(115200);
  IsSerialAvailable = Serial;

  if(IsSerialAvailable) {
    #ifdef AS_TRANSMITTER
      Serial.println(F("working as transmitter"));
    #else
      Serial.println(F("working as receiver"));
    #endif
  }

  //////////////////////////////////////////////////////////////////////
  //inizilizzazione I/0
  //////////////////////////////////////////////////////////////////////
  
  for(int i = 0; i < (int)(sizeof(pins)/(sizeof(pins[0]))); i++) {
    pinMode(pgm_read_byte(&pins[i][0]), pgm_read_byte(&pins[i][1]));
    if(pgm_read_byte(&pins[i][1]) == OUTPUT) {
      digitalWrite(pgm_read_byte(&pins[i][0]), LOW);
    }
  }

  //defined(ENABLE_LIMIT_SW_IRQ) && defined(AS_RECEIVER)
  #ifdef AS_RECEIVER
    //attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), NRF_PacketReadyIRQ, RISING); 
    #ifdef ENABLE_LIMIT_SW_IRQ
      attachInterrupt(digitalPinToInterrupt(LIMIT_SW_IRQ_PIN), LimitSW_interrupt, RISING);
    #endif
  #endif

  #ifdef AS_TRANSMITTER
    attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), NRF_transmitIRQ, RISING);
  #endif


  //////////////////////////////////////////////////////////////////////
  //configurazione TIMER1
  //////////////////////////////////////////////////////////////////////

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
  
  //////////////////////////////////////////////////////////////////////


  interrupts(); // Abilita gli interrupt globali              


  //////////////////////////////////////////////////////////////////////
  // inizializzazione NRF24L01
  //////////////////////////////////////////////////////////////////////

  if (!radio.begin()) {
    setLED_Animation(NRF_ERROR_LED_SEQUENZE);
    
    //ripeto finche non riesco a configurare/trovare il modulo
    do {
      if(IsSerialAvailable) 
        Serial.println(F("radio hardware is not responding!!"));
      
      delay(1000);

      if(IsSerialAvailable) 
        Serial.println(F("new attempt"));
    }
    while(!radio.begin());
    
    setLED_Animation(NO_BLINK_LED_SEQUENZE);
  }

  if(IsSerialAvailable) {
    Serial.println(F("radio hardware detected"));
    Serial.print(F("READING_PIPE: ")); Serial.println((long)READING_PIPE, HEX);
    Serial.print(F("WRITING_PIPE: ")); Serial.println((long)WRITING_PIPE, HEX);
  }

  //////////////////////////////////////////////////////////////////////
  // configurazione NRF24L01
  //////////////////////////////////////////////////////////////////////

  radio.setPALevel(RF24_PA_MAX);              // RF24_PA_MAX is default.
  radio.setChannel(CHANNEL);                  // set the RF channel 
  //radio.setPayloadSize(sizeof(Packet));     // size of the payload 
  radio.setAutoAck(true);                     // enable auto ack
  radio.enableAckPayload();                   // enable ack payload
  radio.setDataRate(RF24_1MBPS);              // set the data rate
  radio.setRetries(0, 15);                    // set the retry counts
  radio.openWritingPipe(WRITING_PIPE);        // always uses pipe 0
  radio.openReadingPipe(1, READING_PIPE);     // using pipe != 0

  #ifdef AS_TRANSMITTER
    radio.maskIRQ(1,0,1);                     // data sent interrupt and data failed interrupt
    radio.stopListening();                    // put radio in TX mode 
  #else
    //radio.maskIRQ(1,1,0);                     // data ready interrupt
    radio.maskIRQ(1,1,1);                     // data ready interrupt
    radio.startListening();                   // put radio in RX mode
    
  #endif

  
  if(IsSerialAvailable) {
    printf_begin();                           // needed only once for printing details
    radio.printPrettyDetails();               // (larger) function that prints human readable data
  }
  //////////////////////////////////////////////////////////////////////

  //imposto lo stato base del led
  #ifdef AS_TRANSMITTER
    setLED_Animation(NO_BLINK_LED_SEQUENZE);
  #else
    setLED_Animation(NRF_CONNECTION_LOST_LED_SEQUENZE);
  #endif
}

void loop() 
{
  #ifdef AS_TRANSMITTER
    //verifico se è giungto il momento di inviare i dati dei pulsanti
    if(millis() - timerPacket >=  60) {
      timerPacket = millis();
      TX_loop();
    }
  #else

    RX_loop();  

    //#if !defined(ENABLE_LIMIT_SW_IRQ)
    //se si attivano i fine corsa
    if(!digitalRead(LEFT_LIMIT_SW) && packet.sw3)
      digitalWrite(LEFT_RELAY_PIN, LOW);

    if(!digitalRead(RIGHT_LIMIT_SW) && packet.sw4)
      digitalWrite(RIGHT_RELAY_PIN, LOW);
    //#endif

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

    //se sono passati 10s dall'ultimo pacchetto inviato => software reset
    if(millis() - lastPacket_sended_time  >= 10000) {
      resetFunc();
    }
  }
#else
  void NRF_PacketReadyIRQ() 
  {

  }

  #if defined(ENABLE_LIMIT_SW_IRQ)
    void LimitSW_interrupt() {
      digitalWrite(RIGHT_RELAY_PIN, LOW);
      digitalWrite(LEFT_RELAY_PIN, LOW);
    }
  #endif
#endif




inline __attribute__((always_inline))
void setLED_Animation(uint8_t index) 
{
  if(index > sizeof(LED_Animations.animations)/sizeof(LED_Animations.animations[0])) {
    //if(serialIsAvailable)
    //  Serial.println(F("Animation index out of range"));    
    return;
  }

  LED_Animations.selectedAnimation = index;
  
  /*if(serialIsAvailable) {
    Serial.print(F("Animation index: "));
    Serial.println(index);
  }*/
}

#ifdef AS_RECEIVER
inline __attribute__((always_inline))
void RX_loop() 
{
  static uint8_t pipe;
  static uint8_t bytes;
  static boolean packetAvailable;
  static uint8_t buffer[32];
  //disabilito gli interrupt per evitare problemi
  //noInterrupts();

  packetAvailable = radio.available(&pipe);


  //se è passato 1s dall'ultimo pacchetto ricevuto
  if(!packetAvailable && (millis() - lastPacket_recived_time >= 1000)) 
  {
    //spengo tutti i relays
    digitalWrite(LEFT_RELAY_PIN,     LOW);
    digitalWrite(RIGHT_RELAY_PIN,    LOW);
    digitalWrite(FORWARD_RELAY_PIN,  LOW);
    digitalWrite(BACKWORD_RELAY_PIN, LOW);

    //accendo il led di "connessione persa" e imposto il lampeggio del LED distatus
    setLED_Animation(NRF_CONNECTION_LOST_LED_SEQUENZE);
    digitalWrite(CONNECTION_LOST_LED_PIN, HIGH);

    //interrupts();

    //aspetto finche non ricevo un nuovo pacchetto
    if(IsSerialAvailable) {
      Serial.println(F("Waiting for transmitter connection"));
    }
      
    while(true) {
      if(radio.available(&pipe)) {
        if(pipe == 1) {
          break;
        }
        else {
          bytes = radio.getPayloadSize();
          radio.read(&buffer, bytes);
        }
      }
      //se non ricevo un nuovo pacchetto dopo 4 secondi => software reset
      if(millis() - lastPacket_recived_time => 4000) {
        resetFunc(); //software reset
      }
    }

    if(IsSerialAvailable) {
      Serial.print(F("transmitter connected on pipe "));
      Serial.println(pipe);
    }
    
    lastPacket_recived_time = millis();
    //spengo il led di "connessione persa"
    digitalWrite(CONNECTION_LOST_LED_PIN, LOW);

  }
  else if(packetAvailable && pipe == 1) {
    lastPacket_recived_time = millis();
    //Serial.print("start: ");Serial.println(lastPacket_recived_time);
    bytes = radio.getPayloadSize();         // get the size of the payload
    radio.read(&packet, bytes);             // fetch payload from FIFO
  
    //per evitare di fare entrambe le cose
    if(packet.sw1 && packet.sw2) {
        packet.sw1 = 0;
        packet.sw2 = 0;
    }

    if(packet.sw3 && packet.sw4) {
        packet.sw3 = 0;
        packet.sw4 = 0;
    }
      
    digitalWrite(FORWARD_RELAY_PIN,  packet.sw1);
    digitalWrite(BACKWORD_RELAY_PIN, packet.sw2);
    
    //I fine corsa sono normalmente aperti => ho sempre 1
    //se il fine corsa è chiuso ho 0 => setto il pin LOW
    digitalWrite(LEFT_RELAY_PIN,   (digitalRead(LEFT_LIMIT_SW) ?  packet.sw3  : LOW));
    digitalWrite(RIGHT_RELAY_PIN,  (digitalRead(RIGHT_LIMIT_SW) ? packet.sw4  : LOW));
    

    if(LED_Animations.selectedAnimation != NO_ERROR_AND_CONNECTED_LED_SEQUENZE) {
      setLED_Animation(NO_ERROR_AND_CONNECTED_LED_SEQUENZE);
      digitalWrite(CONNECTION_LOST_LED_PIN, LOW);
    }

    if(IsSerialAvailable) {
      Serial.print(F("Received "));         Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));   Serial.print(pipe);  // print the pipe number
      Serial.print(F("| sw1 "));  Serial.print(packet.sw1);
      Serial.print(F(" sw2 "));   Serial.print(packet.sw2);
      Serial.print(F(" sw3 "));   Serial.print(packet.sw3);
      Serial.print(F(" sw4 "));   Serial.println(packet.sw4);
    }
    
    //Serial.print("end: "); Serial.println(lastPacket_recived_time);
  }
  else if(packetAvailable){
    bytes = radio.getPayloadSize();
    radio.read(&buffer, bytes);
  }
  else {
    delay(10);
  }
}
#endif

#ifdef AS_TRANSMITTER
inline __attribute__((always_inline))
void TX_loop() 
{
  //static uint8_t *p = (uint8_t *) &packet;

  //leggo i valori dei pulsanti e li salvo in packet
  packet.sw1 = digitalRead(FORWARD_BUTTON_PIN);
  packet.sw2 = digitalRead(BACKWORD_BUTTON_PIN);
  packet.sw3 = digitalRead(LEFT_BUTTON_PIN);
  packet.sw4 = digitalRead(RIGHT_BUTTON_PIN);

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
    
  if(IsSerialAvailable) {
    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);        // print the timer result
      Serial.print(F(" us. Sent: "));

     
      Serial.print(F(" sw1: "));
      Serial.print(packet.sw1);
      Serial.print(F(" | sw2: "));
      Serial.print(packet.sw2);
      Serial.print(F(" | sw3: "));
      Serial.print(packet.sw3);
      Serial.print(F(" | sw4: "));
      Serial.print(packet.sw4);
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
