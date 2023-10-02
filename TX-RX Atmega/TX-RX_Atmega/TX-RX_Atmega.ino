#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

//#define DEBUG
#define AS_TRANSMITTER
//#define AS_RECEIVER

#if defined(AS_TRANSMITTER) && defined(AS_RECEIVER)
  #error Can't be both transmitter and receiver
#endif

#define NODE0_ADDRESS "00001" //0xd2f0f0f0f0
#define NODE1_ADDRESS "00002" //0xe1f0f0f0f0
#define CHANNEL 0x60
#define CSN_PIN 10
#define CE_PIN 8
#define ERROR_LED_PIN 2



RF24 radio(CE_PIN, CSN_PIN);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
long timerLED = 0;
long timerPacket = 0;

#ifdef AS_TRANSMITTER
  uint8_t READING_PIPE[] = {NODE1_ADDRESS}; //leggo dal canale del nodo 1
  uint8_t WRITING_PIPE[] = {NODE0_ADDRESS}; //scrivo sul mio canale
#else
  uint8_t READING_PIPE[] = {NODE0_ADDRESS}; //leggo dal canale del nodo 0
  uint8_t WRITING_PIPE[] = {NODE1_ADDRESS}; //scrivo sul mio canale
  
#endif

uint8_t pins[] = {3,4,5,6,7};
boolean dataAvailable = false;
boolean serialIsAvailable = false;


typedef struct payloadPacket 
{
  uint8_t sw1;
  uint8_t sw2;
  uint8_t sw3;
  uint8_t sw4;
  uint8_t status;

} Packet;

Packet packet;


void setup() 
{
  SPI.begin();
  Serial.begin(115200);

  serialIsAvailable = Serial;

  #ifdef AS_TRANSMITTER
    Serial.println(F("working as transmitter"));

    for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++)
      pinMode(pins[i], INPUT);
    
  #else
    Serial.println(F("working as receiver"));

    for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) {
      pinMode(pins[i], OUTPUT);
      digitalWrite(pins[i], LOW);
    }

    pinMode(ERROR_LED_PIN, OUTPUT);
    digitalWrite(ERROR_LED_PIN, LOW);
  #endif

  
  
  #ifdef DEBUG
    while (!Serial);
  
    if (!radio.begin()) {
      Serial.println(F("radio hardware is not responding!!"));
      while (1) {}  // hold in infinite loop
    }
    Serial.print(F("READING_PIPE: "));
    Serial.println((long)READING_PIPE, HEX);
    Serial.print(F("WRITING_PIPE: "));
    Serial.println((long)WRITING_PIPE, HEX);
  #else
    if (!radio.begin()) {
      if(serialIsAvailable)
        Serial.println(F("radio hardware is not responding!!"));
      while (1) {}  // hold in infinite loop
    }
  #endif

  radio.setPALevel(RF24_PA_MAX);              // RF24_PA_MAX is default.
  radio.setChannel(CHANNEL);                  // set the RF channel 
  radio.setPayloadSize(sizeof(Packet));       // size of the payload 
  radio.setAutoAck(true);                     // enable auto ack
  radio.enableAckPayload();                   // enable ack payload
  radio.setDataRate(RF24_1MBPS);              // set the data rate
  radio.setRetries(0, 15);                    // set the retry counts

  #ifdef AS_RECEIVER
    //radio.maskIRQ(1,1,0);                     // data ready interrupt
    //attachInterrupt(digitalPinToInterrupt(2), getPacketIRQ, RISING);
  #endif

  radio.openWritingPipe(WRITING_PIPE);        // always uses pipe 0
  radio.openReadingPipe(1, READING_PIPE);     // using pipe != 0

  #ifdef AS_TRANSMITTER
    radio.stopListening();    // put radio in TX mode
  #else
    radio.startListening();   // put radio in RX mode
  #endif

  packet.status = 0;
  
  

  #ifdef DEBUG
    printf_begin();             // needed only once for printing details
    radio.printDetails();       // (smaller) function that prints raw register values
    radio.printPrettyDetails(); // (larger) function that prints human readable data
  #endif


}

void loop() 
{
  #ifdef AS_TRANSMITTER

    if(millis() - timerPacket >=  30) {
      timerPacket = millis();

      uint8_t *p = (uint8_t *) &packet;
      for(int i = 0; i < (int)sizeof(pins)/sizeof(pins[0]); i++) 
        p[i] = digitalRead(pins[i]);

      if(millis() - timerLED >=  500) {
        timerLED = millis();
        digitalWrite(3, !digitalRead(3));

        if(packet.status == 0) 
          packet.status = 1;
        else
          packet.status = 0;
      }
      
      
      unsigned long start_timer = micros();                // start the timer
      bool report = radio.write(&packet, sizeof(packet));  // transmit & save the report
      unsigned long end_timer = micros();

      #ifdef DEBUG
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
      #endif
    }
  #else

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
#endif
}


#ifdef AS_RECEIVER
  void getPacketIRQ() {
    dataAvailable = true;
  }
#endif
