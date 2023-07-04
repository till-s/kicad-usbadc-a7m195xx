#!/usr/bin/env python3

# generate XDC from mapping that reflects the naming used
# in the schematic and a package file from the manufacturer
# The pinout file is manually tweaked introducing '#' comments
# to all irrelevant lines...

import re
import io
import sys
import os

here=os.path.abspath(os.path.dirname(__file__)) + "/"

p_skip = re.compile("^[ \t]*($|[#])")

schema = {
   "ulpiClk"           : {"name": "IO_L13P_T2_MRCC_35"},
   "ulpiDat"           : [
                           {"name": "IO_L14N_T2_SRCC_35"},
                           {"name": "IO_L11P_T1_SRCC_35"},
                           {"name": "IO_L10P_T1_AD15P_35"},
                           {"name": "IO_L10N_T1_AD15N_35"},
                           {"name": "IO_L7P_T1_AD6P_35"},
                           {"name": "IO_L7N_T1_AD6N_35"},
                           {"name": "IO_L9P_T1_DQS_AD7P_35"},
                           {"name": "IO_L9N_T1_DQS_AD7N_35"}
                         ],
   "ulpiNxt"           : {"name": "IO_L14P_T2_SRCC_35"},
   "ulpiDir"           : {"name": "IO_L17P_T2_35"},
   "ulpiStp"           : {"name": "IO_L21N_T3_DQS_35"},
   "ulpiRstb"          : {"name": "IO_L22N_T3_35"},
   "i2cScl"            : {"name": "IO_L2P_T0_AD12P_35"},
   "i2cSda"            : {"name": "IO_L2N_T0_AD12N_35"},
   "adcDClk"           : {"name": "IO_L12P_T1_MRCC_15"},
   "adcSync"           : {"name": "IO_L10N_T1_AD11N_15"},
   "adcDORDDR"         : {"name": "IO_L10P_T1_AD11P_15"},
   "adcDatDDR"         : [
                           {"name": "IO_L17N_T2_A25_15"},
                           {"name": "IO_L15N_T2_DQS_ADV_B_15"},
                           {"name": "IO_L18N_T2_A23_15"},
                           {"name": "IO_L21N_T3_DQS_A18_15"},
                           {"name": "IO_L18P_T2_A24_15"},
                           {"name": "IO_L21P_T3_DQS_15"},
                           {"name": "IO_L24N_T3_RS0_15"},
                           {"name": "IO_L23P_T3_FOE_B_15"},
                           {"name": "IO_L22P_T3_A17_15"},
                           {"name": "IO_L23N_T3_FWE_B_15"}
                         ],
   "adcSClk"           : {"name": "IO_L3N_T0_DQS_EMCCLK_14"},
   "adcSDIO"           : {"name": "IO_L7P_T1_D09_14"},
   "adcCSb"            : {"name": "IO_L7N_T1_D10_14"},
   "pgaSClk"           : {"name": "IO_L8N_T1_D12_14"},
   "pgaSDat"           : {"name": "IO_L9N_T1_DQS_D13_14"},
   "pgaCSb"            : [
                           {"name": "IO_L8P_T1_D11_14"},
                           {"name": "IO_L12N_T1_MRCC_14"}
                         ],
   "gpioDat"           : {"name": "IO_L7N_T1_34"},
   "gpioDir"           : {"name": "IO_L8N_T1_34"},
   "pllClk"            : {"name": "IO_L11P_T1_SRCC_15"},
   "led"               : [
                           {"name": "IO_L4P_T0_34"},
                           {"name": "IO_L22P_T3_35"},
                           {"name": "IO_L2N_T0_34"},
                           {"name": "IO_L23N_T3_A02_D18_14"},
                           {"name": "IO_L16N_T2_A15_D31_14"},
                           {"name": "IO_L10N_T1_D15_14"},
                           {"name": "IO_L9P_T1_DQS_14"},
                           {"name": "IO_L10P_T1_D14_14"},
                           {"name": "IO_L16P_T2_CSI_B_14"},
                           {"name": "IO_L8P_T1_AD14P_35"},
                           {"name": "IO_L4N_T0_35"},
                           {"name": "IO_L4P_T0_35"},
                           {"name": "IO_L8N_T1_AD14N_35"}
                         ]
}

class Mapping:

  @staticmethod
  def getDevPinout(fnam):
    pinout={}
    for l in io.open(fnam, 'r'):
      if not p_skip.match(l) is None:
        continue
      wrds = l.split()   
      pinout[wrds[1]] = wrds[0]
    return pinout

  def __init__(self, fnam = 'xc7a35tftg256pkg.txt', defaultIoStd="LVCMOS18", of = sys.stdout):
    self.pinout       = self.getDevPinout( fnam )
    self.defaultIoStd = defaultIoStd
    self.of           = of

  def mapPin(self, port, pin):
    std = self.defaultIoStd
    try:
      std = pin["ioStd"]
    except KeyError:
      pass
    pkgPin = self.pinout[ pin["name"] ]
    print("set_property PACKAGE_PIN {:<8s} [get_ports {{{:s}}}]".format(pkgPin, port), file=self.of)
    print("set_property IOSTANDARD  {:<8s} [get_ports {{{:s}}}]".format(std,    port), file=self.of)

  def genXDC(self, schemaMap = schema):
    print("## This file was automatically generated (gen_pin_xdc.py); DO NOT MODIFY", file=self.of)
    for port,pin in schemaMap.items():
      if ( isinstance(pin, list) ):
         for i in range(len(pin)):
            self.mapPin( "{}[{:d}]".format(port, i), pin[i] )
      else:
        self.mapPin( port, pin )

with io.open("io_pins_xc7a35t_ftg256.xdc", "w") as f:
  m = Mapping( fnam = here + 'xc7a35tftg256pkg.txt', of = f )
  m.genXDC()
