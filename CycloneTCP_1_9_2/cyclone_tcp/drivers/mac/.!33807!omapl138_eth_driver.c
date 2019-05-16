/**
 * @file omapl138_eth_driver.c
 * @brief OMAP-L138 Ethernet MAC controller
 *
 * @section License
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (C) 2010-2019 Oryx Embedded SARL. All rights reserved.
 *
 * This file is part of CycloneTCP Open.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 1.9.2
 **/

//Switch to the appropriate trace level
#define TRACE_LEVEL NIC_TRACE_LEVEL

//Dependencies
#include "soc_omapl138.h"
#include "hw_types.h"
#include "hw_syscfg0_omapl138.h"
#include "hw_emac.h"
#include "hw_emac_ctrl.h"
#include "hw_mdio.h"
#include "cache.h"
#include "interrupt.h"
#include "psc.h"
#include "core/net.h"
#include "drivers/mac/omapl138_eth_driver.h"
#include "debug.h"

//MDIO input clock frequency
#define MDIO_INPUT_CLK 75000000
//MDIO output clock frequency
#define MDIO_OUTPUT_CLK 1000000

//Underlying network interface
static NetInterface *nicDriverInterface;

//IAR EWARM compiler?
#if defined(__ICCARM__)

//Transmit buffer
#pragma data_alignment = 4
#pragma location = ".ram_no_cache"
static uint8_t txBuffer[OMAPL138_ETH_TX_BUFFER_COUNT][OMAPL138_ETH_TX_BUFFER_SIZE];
//Receive buffer
#pragma data_alignment = 4
#pragma location = ".ram_no_cache"
static uint8_t rxBuffer[OMAPL138_ETH_RX_BUFFER_COUNT][OMAPL138_ETH_RX_BUFFER_SIZE];
//Transmit buffer descriptors
#pragma data_alignment = 4
#pragma location = ".ram_cppi"
static Omapl138TxBufferDesc txBufferDesc[OMAPL138_ETH_TX_BUFFER_COUNT];
//Receive buffer descriptors
#pragma data_alignment = 4
#pragma location = ".ram_cppi"
static Omapl138RxBufferDesc rxBufferDesc[OMAPL138_ETH_RX_BUFFER_COUNT];

//Keil MDK-ARM or GCC compiler?
#else

//Transmit buffer
static uint8_t txBuffer[OMAPL138_ETH_TX_BUFFER_COUNT][OMAPL138_ETH_TX_BUFFER_SIZE]
   __attribute__((aligned(4), __section__(".ram_no_cache")));
//Receive buffer
static uint8_t rxBuffer[OMAPL138_ETH_RX_BUFFER_COUNT][OMAPL138_ETH_RX_BUFFER_SIZE]
   __attribute__((aligned(4), __section__(".ram_no_cache")));
//Transmit buffer descriptors
static Omapl138TxBufferDesc txBufferDesc[OMAPL138_ETH_TX_BUFFER_COUNT]
   __attribute__((aligned(4), __section__(".ram_cppi")));
//Receive buffer descriptors
static Omapl138RxBufferDesc rxBufferDesc[OMAPL138_ETH_RX_BUFFER_COUNT]
   __attribute__((aligned(4), __section__(".ram_cppi")));

#endif

//Pointer to the current TX buffer descriptor
static Omapl138TxBufferDesc *txCurBufferDesc;
//Pointer to the current RX buffer descriptor
static Omapl138RxBufferDesc *rxCurBufferDesc;


/**
 * @brief OMAP-L138 Ethernet MAC driver
 **/

const NicDriver omapl138EthDriver =
{
   NIC_TYPE_ETHERNET,
   ETH_MTU,
   omapl138EthInit,
   omapl138EthTick,
   omapl138EthEnableIrq,
   omapl138EthDisableIrq,
   omapl138EthEventHandler,
   omapl138EthSendPacket,
   omapl138EthUpdateMacAddrFilter,
   omapl138EthUpdateMacConfig,
   omapl138EthWritePhyReg,
   omapl138EthReadPhyReg,
   FALSE,
   TRUE,
   TRUE,
   FALSE
};


/**
 * @brief OMAP-L138 Ethernet MAC initialization
 * @param[in] interface Underlying network interface
 * @return Error code
 **/

error_t omapl138EthInit(NetInterface *interface)
{
   error_t error;
   uint32_t channel;
   uint32_t temp;

   //Debug message
   TRACE_INFO("Initializing OMAP-L138 Ethernet MAC...\r\n");

   //Save underlying network interface
   nicDriverInterface = interface;

   //Enable EMAC module
   PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_EMAC,
      PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);

   //Select the interface mode (MII/RMII) and configure pin muxing
   omapl138EthInitGpio(interface);

   //Reset the EMAC control module
   EMAC_CTRL_SOFTRESET_R = EMAC_SOFTRESET_SOFTRESET;
   //Wait for the reset to complete
   while(EMAC_CTRL_SOFTRESET_R & EMAC_SOFTRESET_SOFTRESET);

   //Reset the EMAC module
   EMAC_SOFTRESET_R = EMAC_SOFTRESET_SOFTRESET;
   //Wait for the reset to complete
   while(EMAC_SOFTRESET_R & EMAC_SOFTRESET_SOFTRESET);

   //Calculate the MDC clock divider to be used
   temp = (MDIO_INPUT_CLK / MDIO_OUTPUT_CLK) - 1;

   //Initialize MDIO interface
   MDIO_CONTROL_R = MDIO_CONTROL_ENABLE |
      MDIO_CONTROL_FAULTENB | (temp & MDIO_CONTROL_CLKDIV);

   //PHY transceiver initialization
   error = interface->phyDriver->init(interface);
   //Failed to initialize PHY transceiver?
   if(error)
      return error;

   //Clear the control registers
   EMAC_MACCONTROL_R = 0;
   EMAC_RXCONTROL_R = 0;
   EMAC_TXCONTROL_R = 0;

   //Initialize all 16 header descriptor pointer registers to 0
   for(channel = EMAC_CH0; channel <= EMAC_CH7; channel++)
   {
      //TX head descriptor pointer
      EMAC_TXHDP_R(channel) = 0;
      //TX completion pointer
      EMAC_TXCP_R(channel) = 0;
      //RX head descriptor pointer
      EMAC_RXHDP_R(channel) = 0;
      //RX completion pointer
      EMAC_RXCP_R(channel) = 0;
   }

   //Set the upper 32 bits of the source MAC address
   EMAC_MACSRCADDRHI_R = interface->macAddr.b[0] |
      (interface->macAddr.b[1] << 8) |
      (interface->macAddr.b[2] << 16) |
      (interface->macAddr.b[3] << 24);

   //Set the lower 16 bits of the source MAC address
   EMAC_MACSRCADDRLO_R = interface->macAddr.b[4] |
      (interface->macAddr.b[5] << 8);

   //Write the channel number to the MAC index register
   EMAC_MACINDEX_R = EMAC_CH0;

   //Set the upper 32 bits of the source MAC address
   EMAC_MACADDRHI_R = interface->macAddr.b[0] |
      (interface->macAddr.b[1] << 8) |
      (interface->macAddr.b[2] << 16) |
      (interface->macAddr.b[3] << 24);

   //Set the lower 16 bits of the source MAC address
   temp = interface->macAddr.b[4] |
      (interface->macAddr.b[5] << 8);

   //Use the current MAC address to match incoming packet addresses
   EMAC_MACADDRLO_R = EMAC_MACADDRLO_VALID | EMAC_MACADDRLO_MATCHFILT |
         (EMAC_CH0 << EMAC_MACADDRLO_CHANNEL_SHIFT) | temp;

   //Be sure to program all eight MAC address registers, whether the
   //receive channel is to be enabled or not
   for(channel = EMAC_CH1; channel <= EMAC_CH7; channel++)
   {
      //Write the channel number to the MAC index register
      EMAC_MACINDEX_R = channel;
      //The MAC address is not valid
      EMAC_MACADDRLO_R = (channel << EMAC_MACADDRLO_CHANNEL_SHIFT);
   }

   //Clear the MAC address hash registers
   EMAC_MACHASH1_R = 0;
   EMAC_MACHASH2_R = 0;

   //The RX buffer offset must be initialized to zero
   EMAC_RXBUFFEROFFSET_R = 0;

   //Clear all unicast channels
   EMAC_RXUNICASTCLEAR_R = 0xFF;

   //Accept unicast frames
   EMAC_RXUNICASTSET_R |= (1 << EMAC_CH0);

   //Received CRC is transferred to memory for all channels
   EMAC_RXMBPENABLE_R = EMAC_RXMBPENABLE_RXPASSCRC;

   //Accept broadcast frames
   EMAC_RXMBPENABLE_R |= EMAC_RXMBPENABLE_RXBROADEN |
      (EMAC_CH0 << EMAC_RXMBPENABLE_RXBROADCH_SHIFT);

   //Accept hash matching multicast frames
   EMAC_RXMBPENABLE_R |= EMAC_RXMBPENABLE_RXMULTEN |
      (EMAC_CH0 << EMAC_RXMBPENABLE_RXMULTCH_SHIFT);

   //Register interrupt handlers
   IntRegister(SYS_INT_C0_TX, omapl138EthTxIrqHandler);
   IntRegister(SYS_INT_C0_RX, omapl138EthRxIrqHandler);

   //Set the channel number for the TX interrupt
   IntChannelSet(SYS_INT_C0_TX, OMAPL138_ETH_TX_IRQ_CHANNEL);
   //Set the channel number for the RX interrupt
   IntChannelSet(SYS_INT_C0_RX, OMAPL138_ETH_RX_IRQ_CHANNEL);

   //Clear all unused channel interrupt bits
   EMAC_TXINTMASKCLEAR_R = 0xFF;
   EMAC_RXINTMASKCLEAR_R = 0xFF;

   //Enable the receive and transmit channel interrupt bits
   EMAC_TXINTMASKSET_R = (1 << EMAC_CH0);
   EMAC_RXINTMASKSET_R = (1 << EMAC_CH0);

   //Configure TX and RX buffer descriptors
   omapl138EthInitBufferDesc(interface);

   //Write the RX DMA head descriptor pointer
   EMAC_RXHDP_R(EMAC_CH0) = (uint32_t) rxCurBufferDesc;

   //Enable the receive and transmit DMA controllers
   EMAC_TXCONTROL_R = EMAC_TXCONTROL_TXEN;
   EMAC_RXCONTROL_R = EMAC_RXCONTROL_RXEN;

   //Enable TX and RX
   EMAC_MACCONTROL_R = EMAC_MACCONTROL_GMIIEN;

   //Enable TX and RX completion interrupts
   EMAC_CTRL_CnTXEN_R(EMAC_CORE0) |= (1 << EMAC_CH0);
   EMAC_CTRL_CnRXEN_R(EMAC_CORE0) |= (1 << EMAC_CH0);

   //Accept any packets from the upper layer
   osSetEvent(&interface->nicTxEvent);

   //Successful initialization
   return NO_ERROR;
}


//TMDSLCDK138 board?
#if defined(USE_TMDSLCDK138)

/**
 * @brief GPIO configuration
 * @param[in] interface Underlying network interface
 **/

void omapl138EthInitGpio(NetInterface *interface)
{
   uint32_t temp;

   //Enable GPIO module
   PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO,
      PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);

   //Configure MII_TXD0, MII_TXD1, MII_TXD2, MII_TXD3, MII_COL, MII_TXCLK and MII_TXEN
   temp =  SYSCFG0_PINMUX_R(2) & ~(SYSCFG_PINMUX2_PINMUX2_31_28 |
      SYSCFG_PINMUX2_PINMUX2_27_24 | SYSCFG_PINMUX2_PINMUX2_23_20 |
      SYSCFG_PINMUX2_PINMUX2_19_16 | SYSCFG_PINMUX2_PINMUX2_15_12 |
      SYSCFG_PINMUX2_PINMUX2_11_8 | SYSCFG_PINMUX2_PINMUX2_7_4);

    SYSCFG0_PINMUX_R(2) = temp |
       (SYSCFG_PINMUX2_PINMUX2_31_28_MII_TXD0 << SYSCFG_PINMUX2_PINMUX2_31_28_SHIFT) |
      (SYSCFG_PINMUX2_PINMUX2_27_24_MII_TXD1 << SYSCFG_PINMUX2_PINMUX2_27_24_SHIFT) |
      (SYSCFG_PINMUX2_PINMUX2_23_20_MII_TXD2 << SYSCFG_PINMUX2_PINMUX2_23_20_SHIFT) |
      (SYSCFG_PINMUX2_PINMUX2_19_16_MII_TXD3 << SYSCFG_PINMUX2_PINMUX2_19_16_SHIFT) |
      (SYSCFG_PINMUX2_PINMUX2_15_12_MII_COL << SYSCFG_PINMUX2_PINMUX2_15_12_SHIFT) |
      (SYSCFG_PINMUX2_PINMUX2_11_8_MII_TXCLK << SYSCFG_PINMUX2_PINMUX2_11_8_SHIFT) |
      (SYSCFG_PINMUX2_PINMUX2_7_4_MII_TXEN << SYSCFG_PINMUX2_PINMUX2_7_4_SHIFT);

   //Configure MII_RXD0, MII_RXD1, MII_RXD2, MII_RXD3, MII_CRS, MII_RXER, MII_RXDV and RXCLK
   temp = SYSCFG0_PINMUX_R(3) & ~(SYSCFG_PINMUX3_PINMUX3_31_28 |
      SYSCFG_PINMUX3_PINMUX3_27_24 | SYSCFG_PINMUX3_PINMUX3_23_20 |
      SYSCFG_PINMUX3_PINMUX3_19_16 | SYSCFG_PINMUX3_PINMUX3_15_12 |
      SYSCFG_PINMUX3_PINMUX3_11_8 | SYSCFG_PINMUX3_PINMUX3_7_4 |
      SYSCFG_PINMUX3_PINMUX3_3_0);

   SYSCFG0_PINMUX_R(3) = temp |
      (SYSCFG_PINMUX3_PINMUX3_31_28_MII_RXD0 << SYSCFG_PINMUX3_PINMUX3_31_28_SHIFT) |
      (SYSCFG_PINMUX3_PINMUX3_27_24_MII_RXD1 << SYSCFG_PINMUX3_PINMUX3_27_24_SHIFT) |
      (SYSCFG_PINMUX3_PINMUX3_23_20_MII_RXD2 << SYSCFG_PINMUX3_PINMUX3_23_20_SHIFT) |
      (SYSCFG_PINMUX3_PINMUX3_19_16_MII_RXD3 << SYSCFG_PINMUX3_PINMUX3_19_16_SHIFT) |
      (SYSCFG_PINMUX3_PINMUX3_15_12_MII_CRS << SYSCFG_PINMUX3_PINMUX3_15_12_SHIFT) |
      (SYSCFG_PINMUX3_PINMUX3_11_8_MII_RXER << SYSCFG_PINMUX3_PINMUX3_11_8_SHIFT) |
      (SYSCFG_PINMUX3_PINMUX3_7_4_MII_RXDV << SYSCFG_PINMUX3_PINMUX3_7_4_SHIFT) |
      (SYSCFG_PINMUX3_PINMUX3_3_0_MII_RXCLK << SYSCFG_PINMUX3_PINMUX3_3_0_SHIFT);

    //Configure MDIO and MDCLK
    temp = SYSCFG0_PINMUX_R(4) &  ~(SYSCFG_PINMUX4_PINMUX4_3_0 |
       SYSCFG_PINMUX4_PINMUX4_7_4);

    SYSCFG0_PINMUX_R(4) = temp |
       (SYSCFG_PINMUX4_PINMUX4_7_4_MDIO_D << SYSCFG_PINMUX4_PINMUX4_7_4_SHIFT) |
       (SYSCFG_PINMUX4_PINMUX4_3_0_MDIO_CLK << SYSCFG_PINMUX4_PINMUX4_3_0_SHIFT);

   //Select MII interface mode
   SYSCFG0_CFGCHIP3_R &= ~SYSCFG_CFGCHIP3_RMII_SEL;
}

#endif


/**
 * @brief Initialize buffer descriptor lists
 * @param[in] interface Underlying network interface
 **/

void omapl138EthInitBufferDesc(NetInterface *interface)
{
   uint32_t i;
   uint32_t nextIndex;
   uint32_t prevIndex;

   //Initialize TX buffer descriptor list
   for(i = 0; i < OMAPL138_ETH_TX_BUFFER_COUNT; i++)
   {
      //Index of the next buffer
      nextIndex = (i + 1) % OMAPL138_ETH_TX_BUFFER_COUNT;
      //Index of the previous buffer
      prevIndex = (i + OMAPL138_ETH_TX_BUFFER_COUNT - 1) % OMAPL138_ETH_TX_BUFFER_COUNT;

      //Next descriptor pointer
      txBufferDesc[i].word0 = (uint32_t) NULL;
      //Buffer pointer
      txBufferDesc[i].word1 = (uint32_t) txBuffer[i];
      //Buffer offset and buffer length
      txBufferDesc[i].word2 = 0;
      //Status flags and packet length
      txBufferDesc[i].word3 = 0;

      //Form a doubly linked list
      txBufferDesc[i].next = &txBufferDesc[nextIndex];
      txBufferDesc[i].prev = &txBufferDesc[prevIndex];
   }

   //Point to the very first descriptor
   txCurBufferDesc = &txBufferDesc[0];

   //Mark the end of the queue
   txCurBufferDesc->prev->word3 = EMAC_TX_WORD3_SOP |
      EMAC_TX_WORD3_EOP | EMAC_TX_WORD3_EOQ;

   //Initialize RX buffer descriptor list
   for(i = 0; i < OMAPL138_ETH_RX_BUFFER_COUNT; i++)
   {
      //Index of the next buffer
      nextIndex = (i + 1) % OMAPL138_ETH_RX_BUFFER_COUNT;
      //Index of the previous buffer
      prevIndex = (i + OMAPL138_ETH_RX_BUFFER_COUNT - 1) % OMAPL138_ETH_RX_BUFFER_COUNT;

      //Next descriptor pointer
      rxBufferDesc[i].word0 = (uint32_t) &rxBufferDesc[nextIndex];
      //Buffer pointer
      rxBufferDesc[i].word1 = (uint32_t) rxBuffer[i];
      //Buffer offset and buffer length
      rxBufferDesc[i].word2 = OMAPL138_ETH_RX_BUFFER_SIZE;
      //Status flags and packet length
      rxBufferDesc[i].word3 = EMAC_RX_WORD3_OWNER;

      //Form a doubly linked list
      rxBufferDesc[i].next = &rxBufferDesc[nextIndex];
      rxBufferDesc[i].prev = &rxBufferDesc[prevIndex];
   }

   //Point to the very first descriptor
   rxCurBufferDesc = &rxBufferDesc[0];

   //Mark the end of the queue
   rxCurBufferDesc->prev->word0 = (uint32_t) NULL;
}


/**
 * @brief OMAP-L138 Ethernet MAC timer handler
 *
 * This routine is periodically called by the TCP/IP stack to
 * handle periodic operations such as polling the link state
 *
 * @param[in] interface Underlying network interface
 **/

void omapl138EthTick(NetInterface *interface)
{
   //Handle periodic operations
   interface->phyDriver->tick(interface);

   //Misqueued buffer condition?
   if(rxCurBufferDesc->word3 & EMAC_RX_WORD3_OWNER)
   {
      if(EMAC_RXHDP_R(EMAC_CH0) == 0)
      {
         //The host acts on the misqueued buffer condition by writing the added
         //buffer descriptor address to the appropriate RX DMA head descriptor
         //pointer
         EMAC_RXHDP_R(EMAC_CH0) = (uint32_t) rxCurBufferDesc;
      }
   }
}


/**
 * @brief Enable interrupts
 * @param[in] interface Underlying network interface
 **/

void omapl138EthEnableIrq(NetInterface *interface)
{
   //Enable Ethernet MAC interrupts
   IntSystemEnable(SYS_INT_C0_TX);
   IntSystemEnable(SYS_INT_C0_RX);

   //Enable Ethernet PHY interrupts
   interface->phyDriver->enableIrq(interface);
}


/**
 * @brief Disable interrupts
 * @param[in] interface Underlying network interface
 **/

void omapl138EthDisableIrq(NetInterface *interface)
{
   //Disable Ethernet MAC interrupts
   IntSystemDisable(SYS_INT_C0_TX);
   IntSystemDisable(SYS_INT_C0_RX);

   //Disable Ethernet PHY interrupts
   interface->phyDriver->disableIrq(interface);
}


/**
 * @brief Ethernet MAC transmit interrupt
 **/

void omapl138EthTxIrqHandler(void)
{
   bool_t flag;
   uint32_t status;
   uint32_t temp;
   Omapl138TxBufferDesc *p;

   //Enter interrupt service routine
   osEnterIsr();

   //This flag will be set if a higher priority task must be woken
   flag = FALSE;

   //Clear system interrupt status
   IntSystemStatusClear(SYS_INT_C0_TX);

   //Read the C0TXSTAT register to determine which channels caused the interrupt
   status = EMAC_CTRL_C0TXSTAT_R;

   //Packet transmitted on channel 0?
   if(status & (1 << EMAC_CH0))
   {
      //Point to the buffer descriptor
      p = (Omapl138TxBufferDesc *) EMAC_TXCP_R(EMAC_CH0);

      //Read the status flags
      temp = p->word3 & (EMAC_TX_WORD3_SOP | EMAC_TX_WORD3_EOP |
         EMAC_TX_WORD3_OWNER | EMAC_TX_WORD3_EOQ);

      //Misqueued buffer condition?
      if(temp == (EMAC_TX_WORD3_SOP | EMAC_TX_WORD3_EOP | EMAC_TX_WORD3_EOQ))
      {
         //Check whether the next descriptor pointer is non-zero
         if(p->word0 != 0)
         {
            //The host corrects the misqueued buffer condition by writing the
