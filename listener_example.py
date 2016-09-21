#!/usr/bin/env python3
#import sys
#sys.path.append('/home/rf/modules')
import nRF24L01p

nrf = nRF24L01p.nRF24L01p()
nrf.enableRX()

i = 0
def simple_loop(data):
    global i
    i += 1
    print(data.decode('utf-8'), i)
    if data.decode('utf-8') == 'L1':
        print 'Triggered!'

nrf.readLoop(simple_loop)
