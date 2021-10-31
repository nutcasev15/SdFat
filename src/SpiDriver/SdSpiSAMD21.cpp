/**
 * Copyright (c) 2011-2020 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include "SdSpiDriver.h"

#if defined(SD_USE_CUSTOM_SPI) && defined(__SAMD21G18A__)
/* Use SCK Maximum Frequency Override */
// Reference For Maximum Frequency (12 MHz): https://microchipsupport.force.com/s/article/SPI-max-clock-frequency-in-SAMD-SAMR-devices
#define USE_SAMD21_MAX_SCK_OVERRIDE 1
/* Use SAMD21 DMAC if nonzero */
#define USE_SAMD21_DMAC 1
/* Use critical latency bus priority if nonzero */
#define USE_SAMD21_BUS_CRIT_LAT 0
/* Time in ms for DMA receive timeout */
#define SAMD21_DMA_TIMEOUT 100
/* Number of DMAc channels */
#define NUM_DMA_CHANNELS 1
/* DMAC channel ID */
#define SPI_DMAC_CH 0

/* Determine SERCOM Peripheral to Use */
// In Order:
// MZero, Zero, MKR Zero
#if ((PIN_SPI_MISO == 18u) && (PIN_SPI_MOSI == 21u) && (PIN_SPI_SCK == 20u)) \
  || ((PIN_SPI_MISO == 22u) && (PIN_SPI_MOSI == 23u) && (PIN_SPI_SCK == 24u)) \
  || ((PIN_SPI1_MISO == 29u) && (PIN_SPI1_MOSI == 26u) && (PIN_SPI1_SCK == 27u)) 
  #define SPI_SERCOM SERCOM4
  #define DMAC_SERCOM_TRIG_TX SERCOM4_DMAC_ID_TX
  #define DMAC_SERCOM_TRIG_RX SERCOM4_DMAC_ID_RX
#elif (PIN_SPI_MISO == 30u) && (PIN_SPI_MOSI == 31u) && (PIN_SPI_SCK == 32u)
  // Circuitplay
  #define SPI_SERCOM SERCOM3
  #define DMAC_SERCOM_TRIG_TX SERCOM3_DMAC_ID_TX
  #define DMAC_SERCOM_TRIG_RX SERCOM3_DMAC_ID_RX
#else
  #define SPI_SERCOM SERCOM1
  #define DMAC_SERCOM_TRIG_TX SERCOM1_DMAC_ID_TX
  #define DMAC_SERCOM_TRIG_RX SERCOM1_DMAC_ID_RX
#endif

#if (SPI_INTERFACES_COUNT == 2) && defined(SDCARD_SS_PIN)
  // MKR Zero
  #define SD_SPI_CLASS SPI1
#else
  #define SD_SPI_CLASS SPI
#endif

#if USE_SAMD21_DMAC
/* Allot Channel for Transfers */
volatile DmacDescriptor dmawrbk[NUM_DMA_CHANNELS] __attribute__ ((aligned (16)));
DmacDescriptor dmadesc[NUM_DMA_CHANNELS] __attribute__ ((aligned (16)));

//------------------------------------------------------------------------------
/* Check If DMAC is Configured and in Use */
inline static bool dmac_inuse() {
  return (DMAC->BASEADDR.reg != 0 || DMAC->WRBADDR.reg != 0);
}

//------------------------------------------------------------------------------
/* Stores DMAC Peripheral Usage Status */
static bool dmac_busy = true;

//------------------------------------------------------------------------------
/* Disable DMA Controller */
inline static void dmac_disable() {
  // Disable DMAC
  DMAC->CTRL.bit.DMAENABLE = 0;
#if USE_SAMD21_BUS_CRIT_LAT
  DMAC->QOSCTRL.reg = DMAC_QOSCTRL_DQOS_MEDIUM | DMAC_QOSCTRL_FQOS_MEDIUM | DMAC_QOSCTRL_WRBQOS_MEDIUM;
#endif
}

/* Enable DMA Controller */
inline static void dmac_enable() {
#if USE_SAMD21_BUS_CRIT_LAT
  DMAC->QOSCTRL.reg = DMAC_QOSCTRL_DQOS_HIGH | DMAC_QOSCTRL_FQOS_HIGH | DMAC_QOSCTRL_WRBQOS_HIGH;
#endif
  DMAC->CTRL.bit.DMAENABLE = 1;
}

/* Poll for Transfer Complete */
inline static bool dmac_channel_transfer_done() {
  return !(DMAC->BUSYCH.bit.BUSYCH0 | DMAC->PENDCH.bit.PENDCH0);
}

//------------------------------------------------------------------------------
/* RX Transfer using DMA */
static void dma_receive(uint8_t* dst, uint16_t count) {
  // Configure HW Channel
  DMAC->CHID.bit.ID = DMAC_CHID_ID(SPI_DMAC_CH);
  DMAC->CHCTRLA.bit.ENABLE = 0;
  DMAC->CHCTRLA.bit.SWRST = 1;
  DMAC->CHCTRLB.bit.TRIGSRC = DMAC_SERCOM_TRIG_RX;
  DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_LVL_LVL0;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_SUSP | DMAC_CHINTENSET_TCMPL | DMAC_CHINTENSET_TERR;
  // Configure Channel Descriptor
  dmadesc[SPI_DMAC_CH].BTCTRL.reg = DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_BYTE;
  dmadesc[SPI_DMAC_CH].BTCNT.reg = count;
  dmadesc[SPI_DMAC_CH].SRCADDR.reg = (uint32_t)&SPI_SERCOM->SPI.DATA.reg;
  dmadesc[SPI_DMAC_CH].DSTADDR.reg = (uint32_t)dst;
  dmadesc[SPI_DMAC_CH].DSTADDR.reg += count;
  dmadesc[SPI_DMAC_CH].DESCADDR.reg = 0X0; // This is the only Descriptor
  dmadesc[SPI_DMAC_CH].BTCTRL.reg |= DMAC_BTCTRL_VALID; // Set Descriptor as Valid
  // Configure Transfer Complete Callback
  NVIC_SetPriority(DMAC_IRQn, SERCOM_NVIC_PRIORITY - 1); // Higher Priority than SERCOM IRQs
  //Serial.println(dmadesc[SPI_DMAC_CH].BTCTRL.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].BTCNT.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].SRCADDR.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].DSTADDR.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].DESCADDR.reg);
  // DMAC->CHCTRLA.bit.ENABLE = 1;

  // Load TX Data and Trigger RX
  SPI_SERCOM->SPI.DATA.bit.DATA = 0XFF;
  DMAC->CHCTRLA.bit.ENABLE = 1;
}

//------------------------------------------------------------------------------
/* DMA Interrupt Handler for SPI RX */
void DMA_Handler() {
  // Acknowledge Transfer Complete
  // If Any Other Interrupts are Set, Driver Aborts DMA Transfer
  DMAC->CHID.bit.ID = DMAC_CHID_ID(SPI_DMAC_CH);
  DMAC->CHINTFLAG.bit.TCMPL = 1;
  if (DMAC->CHINTFLAG.bit.TERR | DMAC->CHINTFLAG.bit.SUSP) {
    // Acknowledge Errors
    DMAC->CHINTFLAG.bit.TERR = 1;
    DMAC->CHINTFLAG.bit.SUSP = 1;
    // Reset Channel Configuration
    DMAC->CHCTRLA.bit.ENABLE = 0;
    DMAC->CHCTRLA.bit.SWRST = 1;
    // Invalidate Descriptors
    dmadesc[SPI_DMAC_CH].BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
    dmawrbk[SPI_DMAC_CH].BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
    return;
  }

  // Load TX Data for Next RX Byte
  SPI_SERCOM->SPI.DATA.bit.DATA = 0XFF;
}

//------------------------------------------------------------------------------
/* TX Transfer using DMA */
static void dma_send(const uint8_t* src, uint16_t count) {
  // Configure HW Channel
  DMAC->CHID.bit.ID = DMAC_CHID_ID(SPI_DMAC_CH);
  DMAC->CHCTRLA.bit.ENABLE = 0;
  DMAC->CHCTRLA.bit.SWRST = 1;
  DMAC->CHCTRLB.bit.TRIGSRC = DMAC_SERCOM_TRIG_TX;
  DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_LVL_LVL0;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_SUSP | DMAC_CHINTENSET_TERR;
  // Configure Channel Descriptor
  dmadesc[SPI_DMAC_CH].BTCTRL.reg = DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE;
  dmadesc[SPI_DMAC_CH].BTCNT.reg = count;
  dmadesc[SPI_DMAC_CH].SRCADDR.reg = (uint32_t)src;
  dmadesc[SPI_DMAC_CH].SRCADDR.reg += count;
  dmadesc[SPI_DMAC_CH].DSTADDR.reg = (uint32_t)&SPI_SERCOM->SPI.DATA.reg;
  dmadesc[SPI_DMAC_CH].DESCADDR.reg = 0X0; // This is the only Descriptor
  dmadesc[SPI_DMAC_CH].BTCTRL.reg |= DMAC_BTCTRL_VALID; // Set Descriptor as Valid
  // Disable Transfer Complete Callback
  NVIC_SetPriority(DMAC_IRQn, SERCOM_NVIC_PRIORITY);
  NVIC_DisableIRQ(DMAC_IRQn);
  //Serial.println(dmadesc[SPI_DMAC_CH].BTCTRL.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].BTCNT.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].SRCADDR.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].DSTADDR.reg);
  //Serial.println(dmadesc[SPI_DMAC_CH].DESCADDR.reg);
  // DMAC->CHCTRLA.bit.ENABLE = 1;

  // Trigger TX
  DMAC->CHCTRLA.bit.ENABLE = 1;
}
#endif // USE_SAMD21_DMAC

//------------------------------------------------------------------------------
void SdSpiArduinoDriver::begin(SdSpiConfig spiConfig) {
  (void)spiConfig;
  SD_SPI_CLASS.begin();

#if USE_SAMD21_DMAC
  dmac_busy = dmac_inuse();
#endif // USE_SAMD21_DMAC
}

//------------------------------------------------------------------------------
void SdSpiArduinoDriver::end() {
  m_spi->end();
}

//------------------------------------------------------------------------------
void SdSpiArduinoDriver::activate() {
#if USE_SAMD21_MAX_SCK_OVERRIDE
  // Reinitialize SPI Configuration if Specified SCK Clock > SD_SCK_MHZ(12)
  if(m_spiSettings.getClockFreq() > SD_SCK_MHZ(12))
    m_spiSettings = SPISettings(SD_SCK_MHZ(12), m_spiSettings.getBitOrder(), m_spiSettings.getDataMode());
#endif // USE_SAMD21_MAX_SCK_OVERRIDE
  SD_SPI_CLASS.beginTransaction(m_spiSettings);

#if USE_SAMD21_DMAC
  if(!dmac_busy) {
    // Setup DMAC
    // Start Clocks
    PM->AHBMASK.bit.DMAC_ = 1;
    PM->APBBMASK.bit.DMAC_ = 1;
    dmac_disable();
    // Reset DMAC
    DMAC->CTRL.bit.SWRST = 1;
    DMAC->BASEADDR.bit.BASEADDR = (uint32_t)dmadesc;
    DMAC->WRBADDR.bit.WRBADDR = (uint32_t)dmawrbk;
    DMAC->CTRL.bit.LVLEN0 = 1; // Highest Channel Priority
    dmac_enable();
    // Enable and Configure NVIC
    NVIC_EnableIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, SERCOM_NVIC_PRIORITY - 1); // Higher Priority than SERCOM IRQs
    __enable_irq(); // Enable Global IRQs
    // Initialize Descriptors in SRAM
    memset((void *)dmadesc, 0X00, NUM_DMA_CHANNELS * sizeof(dmadesc[0]));
    memset((void *)dmawrbk, 0X00, NUM_DMA_CHANNELS * sizeof(dmawrbk[0]));
  }
#endif // USE_SAMD21_DMAC
}

//------------------------------------------------------------------------------
void SdSpiArduinoDriver::deactivate() {
  SD_SPI_CLASS.endTransaction();
}

//------------------------------------------------------------------------------
static inline uint8_t spiTransfer(uint8_t b) {
  SPI_SERCOM->SPI.DATA.bit.DATA = b;
  while(!SPI_SERCOM->SPI.INTFLAG.bit.RXC);
  return SPI_SERCOM->SPI.DATA.bit.DATA;
}

//------------------------------------------------------------------------------
// SPI RX Wait Cycle Optimization Table
static const uint8_t rx_wait_clk_tbl[] = {
  0, // SPI SCK @ 24 MHz (Invalid)
  7, // SPI SCK @ 12 MHz
  13, // SPI SCK @ 8 MHz
  18, // SPI SCK @ 6 MHz
  23, // SPI SCK @ 4.8 MHz
  29 // SPI SCK @ 4 MHz
};

//------------------------------------------------------------------------------
uint8_t SdSpiArduinoDriver::receive() {
  return spiTransfer(0XFF);
}

//------------------------------------------------------------------------------
uint8_t SdSpiArduinoDriver::receive(uint8_t* buf, size_t count) {
  int rtn = 0;
  uint8_t wait_clk = (SPI_SERCOM->SPI.BAUD.bit.BAUD > 5) ? 0 : rx_wait_clk_tbl[SPI_SERCOM->SPI.BAUD.bit.BAUD];

#if USE_SAMD21_DMAC
  if (!dmac_busy) {
    /*Serial.println(__FUNCTION__);
    Serial.println("IN");
    Serial.println(DMAC->CTRL.reg);
    Serial.println(DMAC->INTPEND.reg);
    Serial.println(DMAC->INTSTATUS.reg);
    Serial.println(DMAC->BUSYCH.reg);
    Serial.println(DMAC->PENDCH.reg);
    Serial.println(DMAC->ACTIVE.reg);
    Serial.println(DMAC->CHID.reg);
    Serial.println(DMAC->CHCTRLA.reg);
    Serial.println(DMAC->CHCTRLB.reg);
    Serial.println(DMAC->CHINTFLAG.reg);
    Serial.println(DMAC->CHSTATUS.reg);*/
    dma_receive(buf, count);
    /*Serial.println(__FUNCTION__);
    Serial.println("OUT");
    Serial.println(DMAC->CTRL.reg);
    Serial.println(DMAC->INTPEND.reg);
    Serial.println(DMAC->INTSTATUS.reg);
    Serial.println(DMAC->BUSYCH.reg);
    Serial.println(DMAC->PENDCH.reg);
    Serial.println(DMAC->ACTIVE.reg);
    Serial.println(DMAC->CHID.reg);
    Serial.println(DMAC->CHCTRLA.reg);
    Serial.println(DMAC->CHCTRLB.reg);
    Serial.println(DMAC->CHINTFLAG.reg);
    Serial.println(DMAC->CHSTATUS.reg);*/

    uint32_t m = millis();
    while (!dmac_channel_transfer_done()) {
      if ((millis() - m) > SAMD21_DMA_TIMEOUT)  {
        DMAC->CHID.bit.ID = DMAC_CHID_ID(SPI_DMAC_CH);
        DMAC->CHCTRLA.bit.ENABLE = 0;
        rtn = 2;
        break;
      }
    }
    if (SPI_SERCOM->SPI.STATUS.bit.BUFOVF) {
      rtn |= 1;
    }
  } else {
#endif // USE_SAMD21_DMAC
    if (wait_clk > 0) {
      for(size_t i = 0; i < count; i++) {
        SPI_SERCOM->SPI.DATA.bit.DATA = 0XFF;
        while(!SPI_SERCOM->SPI.INTFLAG.bit.DRE);
        __asm__ __volatile__(
        "   mov r7, %0   \n" // Use R7
        "1:              \n"
        "   sub r7, #1   \n" // Substract 1 From R7
        "   bne 1b       \n" // If Result is Not 0, Jump to Label 1
        :                    // No Output
        :  "r" (wait_clk)    // Read wait_clk Into R7
        :  "r7"              // Clobber R7
        );
        buf[i] = SPI_SERCOM->SPI.DATA.bit.DATA;
      }
    } else {
      // SPI SCK Lower Than 4 MHz
      for(size_t i = 0; i < count; i++) {
        SPI_SERCOM->SPI.DATA.bit.DATA = 0XFF;
        while(!SPI_SERCOM->SPI.INTFLAG.bit.RXC);
        buf[i] = SPI_SERCOM->SPI.DATA.bit.DATA;
      }
    }
#if USE_SAMD21_DMAC
  }
#endif // USE_SAMD21_DMAC

  return rtn;
}

//------------------------------------------------------------------------------
// SPI TX Wait Cycle Optimization Table
static const uint8_t tx_wait_clk_tbl[] = {
  0, // SPI SCK @ 24 MHz (Invalid)
  1, // SPI SCK @ 12 MHz
  11, // SPI SCK @ 8 MHz
  17, // SPI SCK @ 6 MHz
  22, // SPI SCK @ 4.8 MHz
  27 // SPI SCK @ 4 MHz
};

//------------------------------------------------------------------------------
void SdSpiArduinoDriver::send(uint8_t data) {
  spiTransfer(data);
}

//------------------------------------------------------------------------------
void SdSpiArduinoDriver::send(const uint8_t* buf , size_t count) {
  uint8_t wait_clk = (SPI_SERCOM->SPI.BAUD.bit.BAUD > 5) ? 0 : tx_wait_clk_tbl[SPI_SERCOM->SPI.BAUD.bit.BAUD];

#if USE_SAMD21_DMAC
  if (!dmac_busy) {
    /*Serial.println(__FUNCTION__);
    Serial.println("IN");
    Serial.println(DMAC->CTRL.reg);
    Serial.println(DMAC->INTPEND.reg);
    Serial.println(DMAC->INTSTATUS.reg);
    Serial.println(DMAC->BUSYCH.reg);
    Serial.println(DMAC->PENDCH.reg);
    Serial.println(DMAC->ACTIVE.reg);
    Serial.println(DMAC->CHID.reg);
    Serial.println(DMAC->CHCTRLA.reg);
    Serial.println(DMAC->CHCTRLB.reg);
    Serial.println(DMAC->CHINTFLAG.reg);
    Serial.println(DMAC->CHSTATUS.reg);*/
    dma_send(buf, count);
    /*Serial.println(__FUNCTION__);
    Serial.println("OUT");
    Serial.println(DMAC->CTRL.reg);
    Serial.println(DMAC->INTPEND.reg);
    Serial.println(DMAC->INTSTATUS.reg);
    Serial.println(DMAC->BUSYCH.reg);
    Serial.println(DMAC->PENDCH.reg);
    Serial.println(DMAC->ACTIVE.reg);
    Serial.println(DMAC->CHID.reg);
    Serial.println(DMAC->CHCTRLA.reg);
    Serial.println(DMAC->CHCTRLB.reg);
    Serial.println(DMAC->CHINTFLAG.reg);
    Serial.println(DMAC->CHSTATUS.reg);*/
    while (!dmac_channel_transfer_done());
  } else {
#endif  // #if USE_SAMD21_DMAC
    if(wait_clk > 0) {
      for(size_t i = 0; i < count; i++) {
        SPI_SERCOM->SPI.DATA.bit.DATA = buf[i];
        while(!SPI_SERCOM->SPI.INTFLAG.bit.DRE);
        __asm__ __volatile__(
        "   mov r7, %0   \n" // Use R7
        "1:              \n"
        "   sub r7, #1   \n" // Substract 1 From R7
        "   bne 1b       \n" // If Result is Not 0, Jump to Label 1
        :                    // No Output
        :  "r" (wait_clk)    // Read wait_clk Into R7
        :  "r7"              // Clobber R7
        );
      }
    } else {
      // SPI SCK Lower Than 4 MHz
      for(size_t i = 0; i < count; i++) {
        SPI_SERCOM->SPI.DATA.bit.DATA = buf[i];
        while(!SPI_SERCOM->SPI.INTFLAG.bit.RXC);
      }
    }
#if USE_SAMD21_DMAC
  }
#endif // USE_SAMD21_DMAC
  // Clear RX Overflow and Buffers
  SPI_SERCOM->SPI.STATUS.bit.BUFOVF = 1;
  SPI_SERCOM->SPI.CTRLB.bit.RXEN = 0;
  while(SPI_SERCOM->SPI.SYNCBUSY.bit.CTRLB); // Wait for Sync
  SPI_SERCOM->SPI.CTRLB.bit.RXEN = 1;
  while(SPI_SERCOM->SPI.SYNCBUSY.bit.CTRLB); // Wait for Sync
}
#endif  // defined(SD_USE_CUSTOM_SPI) && defined(__SAMD21G18A__)
