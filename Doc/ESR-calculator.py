'''
Constants

gain   -> op amp gain factor
rout   -> generator output series resistor
freq   -> generator operating frequency in Hz
gOffset -> gain voltage offset
vOffset -> voltage offset
outOffset -> output resistance offset

vdiv1  -> adc voltage divider lower resistor
vdiv2  -> adc voltage divider higher resistor
vref   -> adc reference voltage
adcres -> adc resolution in bits
'''

vBurden = 0.3
gain = 11
rout = 10
freq = 100000
gr1 = 3.3
gr2 = 47+10
vOffset = -0.4
outOffset = 0.0

vref = 1.0
adcres = 12

import math

adcmaxcount = pow(2, adcres) - 1
gOffset = vBurden * (gr1/(gr1+gr2))

def calcvolts(counts):
    v = (vref * counts) / (adcmaxcount * vdiv1)
    return v

def ampl(volts):
    ampl = volts / gain
    ampl -= gOffset
    print("Amplitude:", ampl)
    return 2 * ampl

def divider(volts):
    a = ampl(volts)
    return vBurden / a if a != 0 else math.inf

def calcESR(volts):
    # r/(r+rout) = 1/div
    # r = 1/div * (r + rout)
    # r - 1/div * r = div * rout
    # r * (div - 1) / div = rout / div
    # r * (div - 1) = rout
    # r = rout / (div - 1)
    r = rout / (divider(volts) - 1)
    print("Impedance:", r)
    r += outOffset
    
    return r

#counts = int(input("Enter counts: "))
#volts = calcvolts(counts)
volts = float(input("Enter voltage: "))
volts += vOffset
esr = calcESR(volts)
mohms = abs(esr) < 1
esr = round(esr, 4)
if mohms: esr = esr * 1000.0
print(str(round(esr, 3)) + " " + ("m" if mohms else "") + "ohms")
