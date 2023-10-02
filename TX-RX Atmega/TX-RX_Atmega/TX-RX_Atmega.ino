#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

//#define DEBUG
//#define AS_TRANSMITTER
#define AS_RECEIVER

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
  #define LEFT_BUTTON_PIN 3
  #define RIGHT_BUTTON_PIN 4
  #define FORWARD_BUTTON_PIN 5 
  #define BACKWORD_BUTTON_PIN 6
  #define STATUS_LED_PIN 7
  #define CSN_PIN 10
  #define CE_PIN 8
#else
  #define LEFT_RELAY_PIN 3
  #define RIGHT_RELAY_PIN 4
  #define FORWARD_RELAY_PIN 5 
  #define BACKWORD_RELAY_PIN 6
  #define STATUS_LED_PIN 7
  #define CSN_PIN 10
  #define CE_PIN 8
  #define CONNECTION_LOST_LED_PIN 9
#endif


typedef struct payloadPacket 
{
  uint8_t sw1;
  uint8_t sw2;
  uint8_t sw3;
  uint8_t sw4;
  uint8_t status;

} Packet;


#ifdef AS_TRANSMITTER
  const uint8_t READING_PIPE[] = {NODE1_ADDRESS}; //leggo dal canale del nodo 1
  const uint8_t WRITING_PIPE[] = {NODE0_ADDRESS}; //scrivo sul mio canale

  const uint8_t pins[][] PROGMEM = {
    {LEFT_BUTTON_PIN, INPUT}, 
    {RIGHT_BUTTON_PIN, INPUT},
    {FORWARD_BUTTON_PIN, INPUT},
    {BACKWORD_BUTTON_PIN, INPUT},
    {STATUS_LED_PIN, OUTPUT},
  };

#else
  const uint8_t READING_PIPE[] = {NODE0_ADDRESS}; //leggo dal canale del nodo 0
  const uint8_t WRITING_PIPE[] = {NODE1_ADDRESS}; //scrivo sul mio canale

  const uint8_t pins[][] PROGMEM = {
    {LEFT_RELAY_PIN, INPUT}, 
    {RIGHT_RELAY_PIN, INPUT},
    {FORWARD_RELAY_PIN, INPUT},
    {BACKWORD_RELAY_PIN, INPUT},
    {STATUS_LED_PIN, OUTPUT},
    {CONNECTION_LOST_LED_PIN, OUTPUT}
  };
#endif

struct Animation
{
  uint8_t step;
  int8_t LED_animation[10];
};

//typedef NRF_ERROR Animation{0, {HIGH,HIGH,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,LOW}}

uint8_t statusLED_index = 0;
struct Animation statusLED_BlinkingType[4];

boolean dataAvailable = false;
boolean serialIsAvailable = false;
long timerLED = 0;
long timerPacket = 0;
RF24 radio(CE_PIN, CSN_PIN);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
Packet packet;


void setup() 
{
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

  if (!radio.begin()) {
    if(serialIsAvailable) {
      Serial.println(F("radio hardware is not responding!!"));
    }

    statusLED_index = 1;

    do {
      delay(1000);
    }
    while(!radio.begin());

    statusLED_index = 0;
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

  #ifdef AS_RECEIVER
    //radio.maskIRQ(1,1,0);                     // data ready interrupt
    //attachInterrupt(digitalPinToInterrupt(2), getPacketIRQ, RISING);
  #endif


  #ifdef AS_TRANSMITTER
    radio.stopListening();    // put radio in TX mode
  #else
    radio.startListening();   // put radio in RX mode
  #endif

  packet.status = 0;
  
  if(serialIsAvailable) {
    Sprintf_begin();             // needed only once for printing details
    //radio.printDetails();       // (smaller) function that prints raw register values
    radio.printPrettyDetails(); // (larger) function that prints human readable data
  }  
}

void loop() 
{
  #ifdef AS_TRANSMITTER
    if(millis() - timerPacket >=  30) {
      timerPacket = millis();
      TX_loop();
    }
  #else
    RX_loop();  
  #endif


#ifdef AS_RECEIVER
  void getPacketIRQ() {
    dataAvailable = true;
  }
#endif


inline void RX_loop() {
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
        digitalWrite(pins[i], p[i]);
      }
      digitalWrite(ERROR_LED_PIN, LOW);

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
        digitalWrite(pins[i], LOW);
      }
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(100);
    }
  }
}

inline void TX_loop() 
{
  static uint8_t *p = (uint8_t *) &packet;

  for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) 
    p[i] = digitalRead(pins[i]);

  if(millis() - timerLED >=  200) {
    timerLED = millis();

    if(packet.status == 0) 
      packet.status = 1;
    else
      packet.status = 0;
  }
  
  
  unsigned long start_timer = micros();                // start the timer
  bool report = radio.write(&packet, sizeof(packet));  // transmit & save the report
  unsigned long end_timer = micros();

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