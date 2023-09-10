import sys
from micropython import const
from nrf24l01 import NRF24L01




#from nrf24l01 import NRF24L01
from machine import SPI, Pin
from time import sleep
import struct
import utime



           
payload_size = 20
# role = "send"
role = "receive"





def main():

    READING_PIPE = b"node2"#"\xd2\xf0\xf0\xf0\xf0"
    WRITING_PIPE = b"node1"#"\xe1\xf0\xf0\xf0\xf0"

    LED = Pin("LED", Pin.OUT)            # Onboard LED
    PIN1 = Pin(10, Pin.OUT)
    PIN2 = Pin(11, Pin.OUT)
    PIN3 = Pin(12, Pin.OUT)
    PIN4 = Pin(13, Pin.OUT)

    SPI_channel = SPI(0,sck=Pin(2), mosi=Pin(3), miso=Pin(4))
    CSN = Pin(5, mode=Pin.OUT, value=1) # Chip Select Not
    CE = Pin(6, mode=Pin.OUT, value=0)  # Chip Enable
    

    NRF = NRF24L01(SPI_channel, CSN, CE, payload_size=payload_size)
    NRF.set_channel(0x4c)
    NRF.open_rx_pipe(1,READING_PIPE)
    NRF.open_tx_pipe(WRITING_PIPE)

    NRF.start_listening()

    t1 = utime.ticks_us()

    print("address: ", " | ", WRITING_PIPE)

    
    while True:

        if utime.ticks_diff(utime.ticks_us(), t1) >= 1000000:
            t1 = utime.ticks_us()
            LED.toggle()
        
        if role == "send":
            data = struct.pack("<I", int(input("Enter a number: ")))
            NRF.send(data)
            
        else:
            if NRF.any():
                package = NRF.recv()
                msg = struct.unpack("s",package)[0].decode()
                print(msg)


       



main()