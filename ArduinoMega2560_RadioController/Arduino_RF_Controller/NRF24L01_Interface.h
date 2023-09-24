#pragma once

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "globalVariable.h"
#include "CardSD.h"


class NRF24L01_Interface
{
    private:
        RF24* radio;
        SD_Card* SDcard;

        boolean RX_READY_IRQ_MASK = true;
        boolean TX_OK_IRQ_MASK = true;
        boolean TX_FAIL_IRQ_MASK = true;

        boolean radioDeviceAvailable;
        uint8_t mode;  //0 write | 1 read

        uint8_t comunicationChannel;
        boolean autoAck;
        uint16_t retriesDelay;
        uint8_t retriesCount;

        rf24_datarate_e dataRate;
        rf24_pa_dbm_e powerLevel;

        uint64_t ReadingPipe;
        uint64_t WritingPipe;

        boolean loadSettings();

        void setReadingMode() {
            radio->startListening();
            mode = 1;
        }
        void setWritingMode() {
            radio->stopListening();
            mode = 0;
        }

        boolean inline setIRQ_Mask() {
            radio->maskIRQ(TX_OK_IRQ_MASK, TX_FAIL_IRQ_MASK, RX_READY_IRQ_MASK);
        }

        boolean inline isInWritingMode() { return mode == 0; }
        boolean inline isInReadingMode() { return mode == 1; }
        
        void setDefaoultSettings() {
            comunicationChannel = 100;
            autoAck = true;
            retriesDelay = 0;
            retriesCount = 15;
            dataRate = RF24_250KBPS;
            powerLevel = RF24_PA_LOW;

            ReadingPipe = 0xE6AA000000;
            WritingPipe = 0xE6AA000010;
        }

    public:
        NRF24L01_Interface(uint8_t ce, uint8_t csn, SD_Card* SDcard);
        boolean init();
        boolean isDeviceAvailable() {
            return radio->isChipConnected();
        }
        void reset();
        void printDetails();

        bool write(const void* buf, uint8_t len, const bool multicast) {
            if(isInReadingMode()) setWritingMode();
            return radio->write(buf,len,multicast);
        }

        bool write(const void* buf, uint8_t len) {
            if(isInReadingMode()) setWritingMode();
            return radio->write(buf,len);
        }

        void read(void* buf, uint8_t len) {
            if(isInWritingMode()) setReadingMode();
            radio->read(buf, len);
        }


        void enableDataReceivedInterrupt() {
            RX_READY_IRQ_MASK = false;
            setIRQ_Mask();
        }

        void disableDataReceivedInterrupt() {
            RX_READY_IRQ_MASK = true;
            setIRQ_Mask();
        }

        void enableDataTransmittedInterrupt() {
           TX_OK_IRQ_MASK = false;
            setIRQ_Mask();
        } 

        void disableDataTransmittedInterrupt() {
            TX_OK_IRQ_MASK = true;
            setIRQ_Mask();
        }

        void enableTransmissionFailedInterrupt() {
            TX_FAIL_IRQ_MASK = false;
            setIRQ_Mask();
        }

        void disableTransmissionFailedInterrupt() {
            TX_FAIL_IRQ_MASK = true;
            setIRQ_Mask();
        }


        uint8_t getPyloadSize() {
            return radio->getPayloadSize();
        }


};