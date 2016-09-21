#!/usr/bin/env python3
#import sys
#sys.path.append('/home/rf/quick2wire-python-api')
import time
import select
from threading import Lock
from quick2wire.spi import SPIDevice, writing, duplex
from quick2wire.gpio import Pin, In, Out, Falling, Rising, Both, pi_header_1

BIT0 = 0x0001
BIT1 = 0x0002
BIT2 = 0x0004
BIT3 = 0x0008
BIT4 = 0x0010
BIT5 = 0x0020
BIT6 = 0x0040
BIT7 = 0x0080
BIT8 = 0x0100

# nRF registers:
CONFIG      = 0x00
EN_AA       = 0x01
EN_RXADDR   = 0x02
SETUP_AW    = 0x03
SETUP_RETR  = 0x04
RF_CH       = 0x05
RF_SETUP    = 0x06
STATUS      = 0x07
OBSERVE_TX  = 0x08
CD          = 0x09
RX_ADDR_P0  = 0x0A
RX_ADDR_P1  = 0x0B
RX_ADDR_P2  = 0x0C
RX_ADDR_P3  = 0x0D
RX_ADDR_P4  = 0x0E
RX_ADDR_P5  = 0x0F
TX_ADDR     = 0x10
RX_PW_P0    = 0x11
RX_PW_P1    = 0x12
RX_PW_P2    = 0x13
RX_PW_P3    = 0x14
RX_PW_P4    = 0x15
RX_PW_P5    = 0x16
ENAA_P5     = 0x20
ENAA_P4     = 0x10
ENAA_P3     = 0x08
ENAA_P2     = 0x04
ENAA_P1     = 0x02
ENAA_P0     = 0x01
ERX_P5      = 0x20
ERX_P4      = 0x10
ERX_P3      = 0x08
ERX_P2      = 0x04
ERX_P1      = 0x02
ERX_P0      = 0x01
DPL_P5      = 0x20
DPL_P4      = 0x10
DPL_P3      = 0x08
DPL_P2      = 0x04
DPL_P1      = 0x02
DPL_P0      = 0x01
EN_DPL      = 0x04
EN_ACK_PAY  = 0x02
EN_DYN_ACK  = 0x01

FIFO_STATUS = 0x17
DYNPD       = 0x1C
FEATURE     = 0x1D
 
READ_REG     = 0x00
WRITE_REG    = 0x20
RESET_STATUS = 0x70
EN_CRC       = 0x08
CRCO         = 0x04
 
WR_TX_PLOAD = 0xA0
R_RX_PAYLOAD = 0x61
 
FLUSH_TX    = 0xE1
FLUSH_RX    = 0xE2
ACTIVATE    = 0x50
R_RX_PL_WID = 0x60
IRQ_MASK    = 0x70
CFGMASK_IRQ = 0x00
IRQ_TXFAILED = 0x10
IRQ_TX      = 0x20
IRQ_RX      = 0x40
RX_FULL     = 0x02
NOP         = 0xFF

PWR_UP = 0x02
PRIM_RX = 0x01
TX_EMPTY = 0x10
RX_EMPTY = 0x01


class nRF24L01p:
    def __init__(self):
        self.nrf24 = SPIDevice(0, 0)
        # set auto-ack on data pipe 0 and 1 (EN_AA)
        self.SET_EN_AA = ENAA_P0 | ENAA_P1
        # use data pipes 0 and 1 (EN_RXADDR)
        self.SET_EN_RXADDR = ERX_P0 | ERX_P1
        # 1Mbps, 0dB and no LNA gain (RF_SETUP)
        self.SET_RF_SETUP = 0x00
        # set dynamic payloads on data pipe 0 and 1. requires corresponding ENAA_Px (DYNPD)
        self.SET_DYNPD = DPL_P0 | DPL_P1
        # enable dynamic payloads
        self.SET_FEATURE = EN_DPL
        # receiver address(RX_ADDR_P0)
        self.RX_ADDR_P1 = [0x47, 0x47, 0x47, 0x47, 0x47]
        # transmitter address (TX_ADDR)
        self.TX_ADDRESS = [0x47, 0x47, 0x47, 0x47, 0x47]
        # set channel to 2400Mhz (RF_CH)
        self.SET_RF_CH = 36
        # set automatic retransmission to 15 retransmit on fail AA, 250uS delay(SETUP_RETR)
        self.SET_SETUP_RETR = BIT7 | BIT6 | BIT5 | BIT4
        # 5 byte address (SETUP_AW)
        self.SET_SETUP_AW = BIT0 | BIT1
        # enable CRC, I think (CONFIG)
        self.SET_CONFIG = EN_CRC
        self.nrf24.speed_hz = 500000
        self.CE_pin = pi_header_1.pin(16, direction=Out)
        with self.CE_pin:
            self.CE_pin.value = 0
            self.CE_pin.value = 1
        # IRQ pin with interrupt on high
        self.IRQ_pin = pi_header_1.pin(22, direction=In, interrupt=Falling)
        # epoll, activated with enableRX and write functions
        self.epoll = None
        self.readLoopFlag = False
        self.is_transmitting = False
        self.rf_status = 0
        self.last_received_pipe = None
        self.lastTXfailed = False
        self.radio_lock = Lock()
        self.setup()

    def doOperation(self, operation):
        toReturn = self.nrf24.transaction(operation)
        return toReturn

    def CFGMASK_CRC(self, a):
        return (a & (EN_CRC | CRCO))

    def maintenanceHook(self):
        ''' from spirilis's Enrf24 '''
        lastirq = self.irq_getreason()
        if lastirq & IRQ_TXFAILED:
            self.lastTXfailed = True
            self.doOperation(writing([FLUSH_TX]))
            self.irq_clear(IRQ_TXFAILED)

        if lastirq & IRQ_TX:
            self.lastTXfailed = False
            self.irq_clear(IRQ_TX)

        if lastirq & IRQ_RX:
            reg = self.doOperation(duplex([FIFO_STATUS, NOP]))[0][1]
            if (reg & RX_FULL) == 0:
                self.irq_clear(IRQ_RX)
            reg = self.doOperation(duplex([R_RX_PL_WID, NOP]))[0]
            self.rf_status = reg[0]
            i = reg[1]
            if i == 0 or i > 32 or ((self.rf_status & 0x0E) >> 1) == 0:
                self.doOperation(writing([FLUSH_RX]))
                self.irq_clear(IRQ_RX)

    def irq_getreason(self):
        return self.doOperation(duplex([STATUS, NOP]))[0][1] & IRQ_MASK

    def irq_clear(self, irq):
        _bytes = [WRITE_REG | STATUS]
        _bytes.append(irq & IRQ_MASK)
        self.doOperation(writing(_bytes))

    def irq_derivereason(self, rf_status):
        return rf_status & IRQ_MASK

    def setAddress(self, addr):
        ''' set RX and TX addresses '''
        with self.radio_lock, self.CE_pin:
            self.RX_ADDR_P1 = addr
            self.TX_ADDRESS = addr
            self.CE_pin.value = 0
            _bytes = [WRITE_REG | RX_ADDR_P1]
            _bytes.extend(self.RX_ADDR_P1)
            self.doOperation(writing(_bytes))
            self.CE_pin.value = 1

    def setChannel(self, channel):
        ''' set rf channel, channel being 2400+x '''
        with self.radio_lock, self.CE_pin:
            self.SET_RF_CH = channel
            self.CE_pin.value = 0
            _bytes = [WRITE_REG | RF_CH]
            _bytes.append(self.SET_RF_CH)
            self.doOperation(writing(_bytes))
            self.CE_pin.value = 1

    def setup(self):
        ''' set initial register values '''
        with self.radio_lock, self.CE_pin:
            self.CE_pin.value = 0
            # clear the CONFIG register
            _bytes = [WRITE_REG | CONFIG]
            _bytes.append(0x00)
            self.doOperation(writing(_bytes))
            # set auto-ack
            _bytes = [WRITE_REG | EN_AA]
            _bytes.append(self.SET_EN_AA)
            self.doOperation(writing(_bytes))
            # set data pipes
            _bytes = [WRITE_REG | EN_RXADDR]
            _bytes.append(self.SET_EN_RXADDR)
            self.doOperation(writing(_bytes))
            # set radio settings
            _bytes = [WRITE_REG | RF_SETUP]
            _bytes.append(self.SET_RF_SETUP)
            self.doOperation(writing(_bytes))
            # reset the status register
            _bytes = [WRITE_REG | STATUS]
            _bytes.append(IRQ_MASK)
            self.doOperation(writing(_bytes))
            # set dynamic payloads
            _bytes = [WRITE_REG | DYNPD]
            _bytes.append(self.SET_DYNPD)
            self.doOperation(writing(_bytes))
            # set features
            _bytes = [WRITE_REG | FEATURE]
            _bytes.append(self.SET_FEATURE)
            self.doOperation(writing(_bytes))
            # flush RX Buffer
            self.doOperation(writing([FLUSH_RX]))
            # flush TX Buffer
            self.doOperation(writing([FLUSH_TX]))
            # reset the status register
            _bytes = [WRITE_REG | STATUS]
            _bytes.append(IRQ_MASK)
            self.doOperation(writing(_bytes))
            _bytes = [WRITE_REG | RF_CH]
            _bytes.append(self.SET_RF_CH)
            self.doOperation(writing(_bytes))
            # set automatic retransmission settings
            _bytes = [WRITE_REG | SETUP_RETR]
            _bytes.append(self.SET_SETUP_RETR)
            self.doOperation(writing(_bytes))
            # set address width
            _bytes = [WRITE_REG | SETUP_AW]
            _bytes.append(self.SET_SETUP_AW)
            self.doOperation(writing(_bytes))
            # set the CONFIG register
            _bytes = [WRITE_REG | CONFIG]
            _bytes.append(self.SET_CONFIG)
            self.doOperation(writing(_bytes))
            # set transmitter address
            _bytes = [WRITE_REG | TX_ADDR]
            _bytes.extend(self.TX_ADDRESS)
            self.doOperation(writing(_bytes))
            # set receiver address on data pipe 1
            _bytes = [WRITE_REG | RX_ADDR_P1]
            _bytes.extend(self.RX_ADDR_P1)
            self.doOperation(writing(_bytes))
            self.CE_pin.value = 1

    def radioState(self):
        with self.radio_lock, self.CE_pin:
            reg = self.doOperation(duplex([CONFIG, NOP]))
            reg = reg[0][1]
            if not reg & PWR_UP:
                return 'ENRF24_STATE_DEEPSLEEP'
            #At this point it's either Standby-I, II or PRX.
            if reg & PRIM_RX:
                if self.CE_pin.value:
                    return 'ENRF24_STATE_PRX'
                #PRIM_RX=1 but CE=0 is a form of idle state.
                return 'ENRF24_STATE_IDLE'
            #Check if TX queue is empty, if so it's idle, if not it's PTX.
            reg = self.doOperation(duplex([FIFO_STATUS, NOP]))
            print(reg)
            reg = ord(reg[0][1])
            if (reg & TX_EMPTY):
                return 'ENRF24_STATE_IDLE'
            return 'ENRF24_STATE_PTX'

    def enableRX(self):
        ''' enables receiver mode '''
        with self.radio_lock, self.CE_pin:
            self.CE_pin.value = 0
            reg = self.doOperation(duplex([FIFO_STATUS, NOP]))[0][1]
            _bytes = [WRITE_REG | CONFIG]
            _bytes.append(CFGMASK_IRQ | self.CFGMASK_CRC(reg) | PWR_UP | PRIM_RX)
            self.doOperation(writing(_bytes))
            if not (reg & PWR_UP):
                # wait 5ms if PWR_UP was off
                time.sleep(0.005)
            self.CE_pin.value = 1

    def read(self):
        ''' checks if received a transmission and returns it as a bytes object. returns None if no transmission was received '''
        with self.radio_lock:
            #self.maintenanceHook() # doesn't work properly
            reg = self.doOperation(duplex([FIFO_STATUS, NOP]))[0]
            self.rf_status = reg[0]
            self.last_received_pipe = (self.rf_status & (1 << 4) - 1) >> 1
            reg = reg[1]
            if reg & RX_EMPTY:
                return None
            reg = self.doOperation(duplex([R_RX_PL_WID, NOP]))[0][1]
            _bytes = [R_RX_PAYLOAD]
            for x in range(0, reg):
                _bytes.append(0x00)
            ret = self.doOperation(duplex(_bytes))
            if self.irq_derivereason(self.rf_status) & IRQ_RX:
                self.irq_clear(IRQ_RX)
            return ret.pop()[1:]

    def write(self, data):
        ''' transmits data and returns true or false whether it failed or not. data needs to be a bytes object '''
        if not len(data):
            return True
        with self.radio_lock:
            self.CE_pin.open()
            is_receiving = False
            self.is_transmitting = True
            self.CE_pin.value = 0
            reg = self.doOperation(duplex([CONFIG, NOP]))[0][1]
            if not (reg & PWR_UP):
                _bytes = [WRITE_REG | CONFIG]
                _bytes.append(CFGMASK_IRQ | self.CFGMASK_CRC(reg) | PWR_UP)
                self.doOperation(writing(_bytes))
                time.sleep(0.005)
            if reg & PRIM_RX:
                is_receiving = True
                _bytes = [WRITE_REG | CONFIG]
                _bytes.append(CFGMASK_IRQ | self.CFGMASK_CRC(reg) | PWR_UP)
                self.doOperation(writing(_bytes))
            _bytes = [WR_TX_PLOAD]
            _bytes.extend(data)
            self.doOperation(writing(_bytes))
            self.CE_pin.value = 1
            time.sleep(0.00003)
            self.CE_pin.value = 0
            if not self.readLoopFlag:
                self.epoll = select.epoll()
                self.IRQ_pin.open()
                self.epoll.register(self.IRQ_pin, select.EPOLLIN | select.EPOLLET)
            events = self.epoll.poll()
            for fileno, event in events:
                if fileno == self.IRQ_pin.fileno():
                    self.maintenanceHook()
            if not self.readLoopFlag:
                self.IRQ_pin.close()
            self.CE_pin.value = 1
            self.is_transmitting = False
            if is_receiving:
                self.CE_pin.close()
                self.radio_lock.release()
                self.enableRX()
                self.radio_lock.acquire()
            else:
                self.CE_pin.close()
            return self.lastTXfailed

    def stopReadLoop(self):
        self.readLoopFlag = False

    def readLoop(self, callback, i=0):
        ''' callback's first parameter is the returned data, i is the number of transmissions to receive (0 = unlimited) '''
        self.readLoopFlag = True
        with self.IRQ_pin:
            self.epoll = select.epoll()
            self.epoll.register(self.IRQ_pin, select.EPOLLIN | select.EPOLLET)
            while self.readLoopFlag:
                events = self.epoll.poll()
                for fileno, event in events:
                    print('IRQ fired!')
                    if fileno == self.IRQ_pin.fileno() and not self.is_transmitting:
                        data = self.read()
                        if data is not None:
                            i-=1
                            if not i:
                                self.stopReadLoop()
                            callback(data)

if __name__ == "__main__":
    nrfm = nRF24L01p()
    nrfm.enableRX()
    i = 0
    def a(data):
        global i
        i+=1
        print(data.decode('utf-8'), i)
        #print(nrfm.rf_status, nrfm.last_received_pipe)
        #print(nrfm.write(data + b'a'))
    nrfm.readLoop(a)
