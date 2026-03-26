/*
  rtl_433_ESP - 433.92 MHz protocols library for ESP32

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with library. If not, see <http://www.gnu.org/licenses/>

  Project Structure

  rtl_433_ESP - Main Class
  decoder.cpp - Wrapper and interface for the rtl_433 classes
  receiver.cpp - Wrapper and interface for RadioLib
  rtl_433 - subset of rtl_433 package

*/
#define RADIOLIB_GODMODE 1
#define RADIOLIB_LOW_LEVEL 1

#include "receiver.h"
#include <rtl_433_ESP.h>

#define RF_SX1276

/*----------------------------- Transceiver SPI Connections -----------------------------*/

#if defined(RF_MODULE_SCK) && defined(RF_MODULE_MISO) && \
    defined(RF_MODULE_MOSI) && defined(RF_MODULE_CS)
#  include <SPI.h>
#  if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
SPIClass newSPI(FSPI);
#  else
SPIClass newSPI(VSPI);
#  endif
#endif

#ifdef RF_SX1276
SX1276 radio = RADIO_LIB_MODULE;
#endif

#ifdef RF_SX1278
SX1278 radio = RADIO_LIB_MODULE;
#endif

#ifdef RF_CC1101
CC1101 radio = RADIO_LIB_MODULE;
#endif

#if defined(RF_SX1276) || defined(RF_SX1278)
uint8_t rtl_433_ESP::OokFixedThreshold = OOK_FIXED_THRESHOLD;
#endif

static uint8_t receiverGpioIrq;

Module* _mod = radio.getMod();

/*----------------------------- End of variable initialization -----------------------------*/

rtl_433_ESP::rtl_433_ESP() {
}

/**
 * @brief Initialize Transceiver and rtl_433 decoders
 * 
 * @param inputPin - GPIO of receiver
 * @param receiveFrequency - receive frequency
 */
void rtl_433_ESP::initReceiver(byte irqInputPin, float receiveFrequency) {
#if defined(RF_SX1276) || defined(RF_SX1278)
  radio.reset();
#endif

  receiverGpioIrq = digitalPinToInterrupt(irqInputPin);
#ifdef MEMORY_DEBUG
  logprintfLn( "Pre initReceiver: %d", ESP.getFreeHeap());
#endif
#ifdef DEMOD_DEBUG
  logprintfLn( " gpio receive pin: %d", irqInputPin);
  logprintfLn( " receive frequency: %f", receiveFrequency);
#endif


#ifdef MEMORY_DEBUG
  logprintfLn( "Post rtlSetup: %d", ESP.getFreeHeap());
#endif

// ESP32 defaults to VSPI, but heltec uses MOSI=27, MISO=19, SCK=5, CS=18
#if defined(RF_MODULE_SCK) && defined(RF_MODULE_MISO) && defined(RF_MODULE_MOSI) && defined(RF_MODULE_CS)
#  ifdef RF_MODULE_INIT_STATUS
  logprintfLn( " SPI Config SCK: %d, MISO: %d, MOSI: %d, CS: %d", RF_MODULE_SCK, RF_MODULE_MISO, RF_MODULE_MOSI, RF_MODULE_CS);
#  endif
  newSPI.begin(RF_MODULE_SCK, RF_MODULE_MISO, RF_MODULE_MOSI, RF_MODULE_CS);
#else
	#error WTF NO MOSI etc?
#endif

  /*----------------------------- Initialize Transceiver -----------------------------*/

#ifdef RF_CC1101
  int state = radio.begin();
#else
  int state = radio.beginFSK();
#endif
  RADIOLIB_STATE(state, "radio.begin()");

  radio.setFrequency(receiveFrequency);

  if (ookModulation) {
    state = radio.setOOK(true);
    RADIOLIB_STATE(state, "setOOK");
  } else {
    state = radio.setOOK(false);
    RADIOLIB_STATE(state, "setFSK");
  }

  state = radio.setCrcFiltering(false);
  RADIOLIB_STATE(state, "setCrcFiltering off");

#ifdef RF_CC1101
  if (ookModulation) {
    // set mode to standby
    radio.SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);

    state = radio.SPIsetRegValue(RADIOLIB_CC1101_REG_PKTLEN, 0);
    RADIOLIB_STATE(state, "set PKTLEN");

    // Settings borrowed from lsatan

    state = radio.SPIsetRegValue(RADIOLIB_CC1101_REG_AGCCTRL2, 0xc7);
    RADIOLIB_STATE(state, "set AGCCTRL2");

    state = radio.SPIsetRegValue(RADIOLIB_CC1101_REG_MDMCFG3, 0x93); // Data rate
    RADIOLIB_STATE(state, "set MDMCFG3");

    state = radio.SPIsetRegValue(RADIOLIB_CC1101_REG_MDMCFG4, 0x07); // Bandwidth
    RADIOLIB_STATE(state, "set MDMCFG4");
  } else {
    // From https://github.com/matthias-bs/BresserWeatherSensorReceiver/issues/41#issuecomment-1458166772
    // radio.begin(868.3, 17.24, 40, 270, 10, 32);
    // carrier frequency:                   868.3 MHz
    // bit rate:                            17.24 kbps
    // frequency deviation:                 40 kHz
    // Rx bandwidth:                        270.0 kHz (CC1101) / 250 kHz (SX1276)
    // output power:                        10 dBm
    // preamble length:                     32 bits

    state = radio.setFrequencyDeviation(40); //
    RADIOLIB_STATE(state, "setFrequencyDeviation");

    state = radio.setBitRate(17.24);
    RADIOLIB_STATE(state, "setBitRate");

    state = radio.setRxBandwidth(270); // Sweet spot found from testing
    RADIOLIB_STATE(state, "setRxBandwidth");
  }
  state = radio.disableSyncWordFiltering(false);
  RADIOLIB_STATE(state, "disableSyncWordFiltering");
#endif

#if defined(RF_SX1276) || defined(RF_SX1278)
  if (ookModulation) {
    state = radio.setDataShapingOOK(2); // Default 0 ( 0, 1, 2 )
    RADIOLIB_STATE(state, "setDataShapingOOK");

    state = radio.setOokThresholdType(
        RADIOLIB_SX127X_OOK_THRESH_PEAK); // Peak is default
    RADIOLIB_STATE(state, "OOK Thresh PEAK");

    state = radio.setOokPeakThresholdDecrement(
        RADIOLIB_SX127X_OOK_PEAK_THRESH_DEC_1_1_CHIP); // default
    RADIOLIB_STATE(state, "OOK PEAK Thresh Decrement");

    state = radio.setOokPeakThresholdStep(
        RADIOLIB_SX127X_OOK_PEAK_THRESH_STEP_0_5_DB); // default
    RADIOLIB_STATE(state, "Ook Peak Threshold Step");

    state = radio.setOokFixedOrFloorThreshold(
        OokFixedThreshold); // Default 0x0C RADIOLIB_SX127X_OOK_FIXED_THRESHOLD
    RADIOLIB_STATE(state, "OokFixedThreshold");

    state = radio.setBitRate(1.2);
    logprintfLn( "setBitRate 1.2k");
    RADIOLIB_STATE(state, "setBitRate");

    state = radio.setRxBandwidth(SX127X_RXBANDWIDTH); // Lowering to 125 lowered number of received signals
    logprintfLn( "SX127X_RXBANDWIDTH = %d", SX127X_RXBANDWIDTH);
    RADIOLIB_STATE(state, "setRxBandwidth");

    //https://www.google.com/search?client=ubuntu-sn&channel=fs&q=Acurite-5n1+vs+Acurite-511

  } 
  else
  {
    // From https://github.com/matthias-bs/BresserWeatherSensorReceiver/issues/41#issuecomment-1458166772
    // radio.begin(868.3, 17.24, 40, 270, 10, 32);
    // carrier frequency:                   868.3 MHz
    // bit rate:                            17.24 kbps
    // frequency deviation:                 40 kHz
    // Rx bandwidth:                        270.0 kHz (CC1101) / 250 kHz (SX1276)
    // output power:                        10 dBm
    // preamble length:                     32 bits

    state = radio.setFrequencyDeviation(40); //
    RADIOLIB_STATE(state, "setFrequencyDeviation");

    state = radio.setBitRate(17.24);
    RADIOLIB_STATE(state, "setBitRate");

    state = radio.setRxBandwidth(
        83); // Lowering to 125 lowered number of received signals
    RADIOLIB_STATE(state, "setRxBandwidth");
  }
  state = radio.setRSSIConfig(RADIOLIB_SX127X_RSSI_SMOOTHING_SAMPLES_2, RADIOLIB_SX127X_OOK_AVERAGE_OFFSET_0_DB); // Default 8 ( 2, 4, 8, 16, 32,
  // 64, 128, 256)
  RADIOLIB_STATE(state, "RSSI Smoothing");

  state = _mod->SPIsetRegValue(RADIOLIB_SX127X_REG_PREAMBLE_DETECT,
                               RADIOLIB_SX127X_PREAMBLE_DETECTOR_OFF);
  RADIOLIB_STATE(state, "preamble detect off");

  state = radio.setDirectSyncWord(0, 0); // Disable
  RADIOLIB_STATE(state, "setDirectSyncWord");
  if (ookModulation) {
    state = radio.disableBitSync();
    RADIOLIB_STATE(state, "disableBitSync");
  }
#endif

#ifdef MEMORY_DEBUG
  logprintfLn( "Post config receivers: %d", ESP.getFreeHeap());
#endif

  // Receviers configured, start reception

#if defined(RF_SX1276) || defined(RF_SX1278)
  state = radio.receiveDirect();
#else
  state = radio.receiveDirectAsync();
#endif
  RADIOLIB_STATE(state, "receiveDirect");

#ifdef RESOURCE_DEBUG
  logprintfLn( "rtl_433_ReceiverTask_Stack %d", rtl_433_ReceiverTask_Stack);
#endif

#ifdef RF_MODULE_INIT_STATUS
  getModuleStatus();
#endif

}

/**
 * @brief Main pulse receiver logic
 * 
 */
void ICACHE_RAM_ATTR rtl_433_ESP::interruptHandler() 
{
}

/**
 * @brief Enable signal receiver logic
 * 
 * @param inputPin 
 */
void rtl_433_ESP::enableReceiver() {
  if (receiverGpioIrq >= 0) {
    pinMode(receiverGpioIrq, INPUT);
    
	logprintfLn( "Pin %d has interrupt handler %d", receiverGpioIrq);
    attachInterrupt((uint8_t)receiverGpioIrq, interruptHandler, CHANGE);
    _enabledReceiver = true;
     
	setDebug(3);
   }
}

/**
 * @brief Disable receiver logic, and pulse receiver
 * 
 */
void rtl_433_ESP::disableReceiver() {
  _enabledReceiver = false;
  detachInterrupt((uint8_t)receiverGpioIrq);
}
  
/**
 * @brief Client callback to receive decoded signals
 * 
 * @param callback 
 * @param messageBuffer 
 * @param bufferSize 
 */
 /**
 * @brief Set delta applied to average RSSI level for determining start and end of signal
 * 
 * @param newRssi 
 */
void rtl_433_ESP::setRSSIThreshold(int newRssi) {
  rssiThresholdDelta = newRssi;
#ifndef AUTORSSITHRESHOLD
  logprintfLn( "RSSI Threshold not available: %d", rssiThresholdDelta);
#else
  logprintfLn( "Setting RSSI Threshold Delta to: %d",
              rssiThresholdDelta);
#endif
}

/**
 * @brief set OOK Threshold
 * 
 */
#if defined(RF_SX1276) || defined(RF_SX1278)
void rtl_433_ESP::setOOKThreshold(int newOokThreshold) {
  OokFixedThreshold = newOokThreshold;
#  ifdef REGOOKFIX_DEBUG
  logprintfLn( "Setting setOokFixedOrFloorThreshold to: %d",
              OokFixedThreshold);
#  endif

  int state = radio.setOokFixedOrFloorThreshold(OokFixedThreshold);
  RADIOLIB_STATE(state, "setOokFixedThreshold");
}
#endif

/**
 * @brief This does not work
 * 
 * @param debug 
 */

  /**
   * rtlDebug
   * 0=normal
   * 1=verbose
   * 2=verbose decoders
   * 3=debug decoders
   * 4=trace decoding
   */
   
void rtl_433_ESP::setDebug(int debug) {
  rtlVerbose = debug;
  logprintfLn( "Setting rtl_433 debug to: %d", rtlVerbose);
}

/**
 * @brief Send RTL_433_ESP status to serial port and client. Also send to serial port transceiver status.
 * 
 * @param status 
 */
void rtl_433_ESP::getStatus() {
  alogprintfLn( " ");
}

/****************************************************************
 *FUNCTION NAME:RSSI Level - Replacement RSSI function for the one in RadioLib
 *that doesn't work in OOK async mode for a FUNCTION     :Calculating the RSSI
 *Level INPUT        :none OUTPUT       :none
 ****************************************************************/
int rtl_433_ESP::_getRSSI(void) {
  int rssi;
#ifdef RF_CC1101
  rssi = radio.getRSSI();
#elif RADIOLIB_VERSION_MAJOR >= 6
  rssi = radio.getRSSI(true, true);
#else
  rssi = radio.getRSSI(true);
#endif
  return rssi;
}

/**
 * Send to serial output current transceiver status
 *
 */
void rtl_433_ESP::getModuleStatus() {
#ifdef RF_CC1101
  alogprintfLn( "----- CC1101 Status -----");
  alogprintfLn( "CC1101_MDMCFG1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_MDMCFG1));
  alogprintfLn( "CC1101_MDMCFG2: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_MDMCFG2));
  alogprintfLn( "CC1101_MDMCFG3: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_MDMCFG3));
  alogprintfLn( "CC1101_MDMCFG4: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_MDMCFG4));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_DEVIATN: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_DEVIATN));
  alogprintfLn( "CC1101_AGCCTRL0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_AGCCTRL0));
  alogprintfLn( "CC1101_AGCCTRL1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_AGCCTRL1));
  alogprintfLn( "CC1101_AGCCTRL2: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_AGCCTRL2));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_IOCFG0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_IOCFG0));
  alogprintfLn( "CC1101_IOCFG1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_IOCFG1));
  alogprintfLn( "CC1101_IOCFG2: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_IOCFG2));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_FIFOTHR: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FIFOTHR));
  alogprintfLn( "CC1101_SYNC0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_SYNC0));
  alogprintfLn( "CC1101_SYNC1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_SYNC1));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_PKTLEN: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_PKTLEN));
  alogprintfLn( "CC1101_PKTCTRL0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_PKTCTRL0));
  alogprintfLn( "CC1101_PKTCTRL1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_PKTCTRL1));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_ADDR: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_ADDR));
  alogprintfLn( "CC1101_CHANNR: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_CHANNR));
  alogprintfLn( "CC1101_FSCTRL0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FSCTRL0));
  alogprintfLn( "CC1101_FSCTRL1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FSCTRL1));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_FREQ0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FREQ0));
  alogprintfLn( "CC1101_FREQ1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FREQ1));
  alogprintfLn( "CC1101_FREQ2: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FREQ2));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_MCSM0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_MCSM0));
  alogprintfLn( "CC1101_MCSM1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_MCSM1));
  alogprintfLn( "CC1101_MCSM2: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_MCSM2));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_FOCCFG: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FOCCFG));

  alogprintfLn( "CC1101_BSCFG: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_BSCFG));
  alogprintfLn( "CC1101_WOREVT0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_WOREVT0));
  alogprintfLn( "CC1101_WOREVT1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_WOREVT1));
  alogprintfLn( "CC1101_WORCTRL: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_WORCTRL));
  alogprintfLn( "CC1101_FREND0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FREND0));
  alogprintfLn( "CC1101_FREND1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FREND1));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_FSCAL0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FSCAL0));
  alogprintfLn( "CC1101_FSCAL1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FSCAL1));
  alogprintfLn( "CC1101_FSCAL2: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FSCAL2));
  alogprintfLn( "CC1101_FSCAL3: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_FSCAL3));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_RCCTRL0: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_RCCTRL0));
  alogprintfLn( "CC1101_RCCTRL1: 0x%.2x",
               radio.SPIreadRegister(RADIOLIB_CC1101_REG_RCCTRL1));
  alogprintfLn( "-------------------------");
  alogprintfLn( "CC1101_PARTNUM: 0x%.2x",
               radio.SPIgetRegValue(RADIOLIB_CC1101_REG_PARTNUM));
  alogprintfLn( "CC1101_VERSION: 0x%.2x",
               radio.SPIgetRegValue(RADIOLIB_CC1101_REG_VERSION));
  alogprintfLn( "CC1101_MARCSTATE: 0x%.2x",
               radio.SPIgetRegValue(RADIOLIB_CC1101_REG_MARCSTATE));
  alogprintfLn( "CC1101_PKTSTATUS: 0x%.2x",
               radio.SPIgetRegValue(RADIOLIB_CC1101_REG_PKTSTATUS));
  alogprintfLn( "CC1101_RXBYTES: 0x%.2x",
               radio.SPIgetRegValue(RADIOLIB_CC1101_REG_RXBYTES));

  alogprintfLn( "----- CC1101 Status -----");
#endif
#if defined(RF_SX1276) || defined(RF_SX1278)

  alogprintfLn( "----- SX127x Status -----");

  OokFixedThreshold = _mod->SPIreadRegister(RADIOLIB_SX127X_REG_OOK_FIX);

  alogprintfLn( "RegOpMode: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_OP_MODE));
  alogprintfLn( "RegPacketConfig1: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_PACKET_CONFIG_2));
  alogprintfLn( "RegPacketConfig2: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_PACKET_CONFIG_2));
  alogprintfLn( "RegBitrateMsb: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_BITRATE_MSB));
  alogprintfLn( "RegBitrateLsb: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_BITRATE_LSB));
  alogprintfLn( "RegRxBw: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_RX_BW));
  alogprintfLn( "RegAfcBw: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_AFC_BW));
                if (ookModulation) {
  alogprintfLn( "-------------------------");
  alogprintfLn( "RegOokPeak: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_OOK_PEAK));
  alogprintfLn( "RegOokFix: 0x%.2x", OokFixedThreshold);
  alogprintfLn( "RegOokAvg: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_OOK_AVG));
                }
  alogprintfLn( "-------------------------");
  alogprintfLn( "RegLna: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_LNA));
  alogprintfLn( "RegRxConfig: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_RX_CONFIG));
  alogprintfLn( "RegRssiConfig: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_RSSI_CONFIG));

  alogprintfLn( "-------------------------");
  alogprintfLn( "RegDioMapping1: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_DIO_MAPPING_1));

 if (!ookModulation) {
  alogprintfLn( "----------- FSK --------------");
  alogprintfLn( "FDEV_MSB: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_FDEV_MSB));
  alogprintfLn( "FDEV_LSB: 0x%.2x",
               _mod->SPIreadRegister(RADIOLIB_SX127X_REG_FDEV_LSB));
 }
  alogprintfLn( "----- SX127x Status -----");

#endif
}

/**
 * Functions used only during testing
 *
 */
#if defined(setBitrate) || defined(setFreqDev) || defined(setRxBW)
int16_t rtl_433_ESP::setFrequencyDeviation(float value) {
  return radio.setFrequencyDeviation(value);
}

int16_t rtl_433_ESP::receiveDirect() {
#if defined(RF_SX1276) || defined(RF_SX1278)
  return radio.receiveDirect();
#else
  return radio.receiveDirectAsync();
#endif
}

int16_t rtl_433_ESP::setBitRate(float value) {
  return radio.setBitRate(value);
}

int16_t rtl_433_ESP::setRxBandwidth(float value) {
  return radio.setRxBandwidth(value);
}
#endif
