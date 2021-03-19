/*
 * SX1276.c
 *
 *  Created on: 30 set 2020
 *      Author: Alessio
 */

#include "SX1276.h"
#include "time.h"


static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

SX1276_set SX1276;


typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth;

const FskBandwidth FskBandwidths[] =
{
    {2600  , 0x17}, {3100  , 0x0F}, {3900  , 0x07}, {5200  , 0x16}, {6300  , 0x0E},
    {7800  , 0x06}, {10400 , 0x15}, {12500 , 0x0D}, {15600 , 0x05}, {20800 , 0x14},
    {25000 , 0x0C}, {31300 , 0x04}, {41700 , 0x13}, {50000 , 0x0B}, {62500 , 0x03},
    {83333 , 0x12}, {100000, 0x0A}, {125000, 0x02}, {166700, 0x11}, {200000, 0x09},
    {250000, 0x01}, {300000, 0x00}
};


void SPI_sendByte(uint8_t data)
{
	delay(5);

	while(((SPI0->S) & (1 << 5)) == 0);

	SPI0->D = data;

	while(((SPI0->S) & (1 << 5)) == 0);
}

uint8_t SPI_readByte(void)
{
	while(((SPI0->S) & (1 << 7)) == 0);

	uint8_t data = SPI0->D;

	while(((SPI0->S) & (1 << 7)) == 0);

	return data;
}

void SX1276_writeRegister(uint32_t address, uint8_t data)
{
	GPIOD->PCOR |= (1 << 0); // NSS (CS) = 0;

	SPI_sendByte(address | 0x80);

	SPI_sendByte(data);

	GPIOD->PSOR |= (1 << 0); // NSS (CS) = 1;
}

uint8_t SX1276_readRegister(uint32_t address)
{
	GPIOD->PCOR |= (1 << 0); // NSS (CS) = 0;

	SPI_sendByte(address | 0x7F);

	uint8_t data = SPI_readByte();

	GPIOD->PSOR |= (1 << 0); // NSS (CS) = 1;

	return data;
}

void SX1276_writeMoreThanOneData(uint32_t address, uint8_t *data, uint8_t size)
{
	uint8_t i;

	GPIOD->PCOR |= (1 << 0); // NSS (CS) = 0;

	SPI_sendByte(address | 0x80); // Long range mode & address

	for(i = 0; i < size; i++)
	{
		SPI_sendByte(data[i]);
	}

	GPIOD->PSOR |= (1 << 0); // NSS (CS) = 1;
}

void SX1276_readMoreThanOneData(uint32_t address, uint8_t *data, uint8_t size)
{
	uint8_t i;

	GPIOD->PCOR |= (1 << 0); // NSS (CS) = 0;

	SPI_sendByte(address | 0x7F); // Long range mode & address

	for(i = 0; i < size; i++)
	{
		data[i] = SPI_readByte();
	}

	GPIOD->PSOR |= (1 << 0); // NSS (CS) = 1;
}

uint8_t SX1276_getFskBandwidthRegValue(uint32_t bandwidth)
{
    uint8_t i;

    for(i = 0; i < (sizeof(FskBandwidth) / sizeof(FskBandwidth)) - 1; i++)
    {
        if((bandwidth >= FskBandwidths[i].bandwidth) && (bandwidth < FskBandwidths[i + 1].bandwidth))
        {
            return FskBandwidths[i].RegValue;
        }
    }

    // ERROR: Value not found
    while(1);
}

SX1276_state SX1276_getStatus(void)
{
    return SX1276.State;
}


void SX1276_setChannel(uint32_t freq)
{
	freq = (uint32_t)((double)freq / (double)FREQ_STEP);

    SX1276_writeRegister(REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    SX1276_writeRegister(REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
	SX1276_writeRegister(REG_FRFLSB, (uint8_t)(freq & 0xFF));
}

void SX1276_setOpMode(uint8_t mode)
{
//	SX1276_writeRegister(REG_OPMODE, (SX1276_readRegister(REG_OPMODE) & RF_OPMODE_MASK) | mode);

	SX1276_writeRegister(REG_OPMODE, mode);
}

void SX1276_setModem(SX1276_modemType modem)
{
	if((SX1276_readRegister(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) != 0)
	{
		SX1276.Modem = MODEM_LORA;
	}
	else
	{
		SX1276.Modem = MODEM_FSK;
	}

    switch(modem)
    {
    	default:

    	case MODEM_FSK:

    		SX1276_setOpMode(RF_OPMODE_SLEEP);

    		SX1276_writeRegister(REG_OPMODE, (SX1276_readRegister(REG_OPMODE) | RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);
    		SX1276_writeRegister(REG_DIOMAPPING1, 0x00);
    		SX1276_writeRegister(REG_DIOMAPPING2, 0x30);

			break;

    	case MODEM_LORA:

    		SX1276_setOpMode(RF_OPMODE_SLEEP);

    		SX1276_writeRegister(REG_OPMODE, (SX1276_readRegister(REG_OPMODE) | RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);
    		SX1276_writeRegister(REG_DIOMAPPING1, 0x00);
    		SX1276_writeRegister( REG_DIOMAPPING2, 0x00);

			break;
    }
}

void SX1276_setSleep(void)
{
	SX1276_setOpMode(RF_OPMODE_SLEEP);

    SX1276.State = RF_IDLE;
}

void SX1276_setMaxPayloadLength(uint8_t maxLength)
{
    switch(SX1276.Modem)
    {
    	case MODEM_FSK:

    		if(SX1276.Fsk.FixLength == false)
    		{
    			SX1276_writeRegister(REG_PAYLOADLENGTH, maxLength);
    		}
    		break;

    	case MODEM_LORA:

    		SX1276_writeRegister(REG_LR_PAYLOADMAXLENGTH, maxLength);
    		break;
    }
}

void SX1276_setStandby(void)
{
    SX1276_setOpMode(RF_OPMODE_STANDBY);

    SX1276.State = RF_IDLE;
}

void SX1276_setRx(void)
{
    bool rxContinuous = false;

    switch(SX1276.Modem )
    {
    	case MODEM_FSK:
        {
            rxContinuous = SX1276.Fsk.RxContinuous;

            // DIO0 = PayloadReady
            // DIO1 = FifoLevel
            // DIO2 = SyncAddr
            // DIO3 = FifoEmpty
            // DIO4 = Preamble
            // DIO5 = ModeReady

            SX1276_writeRegister(REG_DIOMAPPING1, (SX1276_readRegister(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK)
            				| RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_11);

            SX1276_writeRegister(REG_DIOMAPPING2, (SX1276_readRegister(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK & RF_DIOMAPPING2_DIO4_11)
            				| RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

            SX1276.FskPacketHandler.FifoThresh = SX1276_readRegister(REG_FIFOTHRESH) & 0x3F;

            SX1276_writeRegister(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

            SX1276.FskPacketHandler.PreambleDetected = false;
            SX1276.FskPacketHandler.SyncWordDetected = false;
            SX1276.FskPacketHandler.NbBytes = 0;
            SX1276.FskPacketHandler.Size = 0;
        }
        break;

    	case MODEM_LORA:
        {
            if(SX1276.LoRa.IqInverted == true)
            {
                SX1276_writeRegister(REG_LR_INVERTIQ, ( (SX1276_readRegister(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1276_writeRegister(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            }
            else
            {
                SX1276_writeRegister(REG_LR_INVERTIQ, ( (SX1276_readRegister(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276_writeRegister(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal

            if(SX1276.LoRa.Bandwidth < 9)
            {
                SX1276_writeRegister(REG_LR_DETECTOPTIMIZE, SX1276_readRegister(REG_LR_DETECTOPTIMIZE) & 0x7F);
                SX1276_writeRegister(REG_LR_IFFREQ2, 0x00);

                switch(SX1276.LoRa.Bandwidth)
                {
                	case 0: // 7.8 kHz
                		SX1276_writeRegister(REG_LR_IFFREQ1, 0x48);
                		SX1276_setChannel(SX1276.Channel + 7810);
                    break;

					case 1: // 10.4 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x44);
						SX1276_setChannel(SX1276.Channel + 10420);
					break;

					case 2: // 15.6 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x44);
						SX1276_setChannel(SX1276.Channel + 15620);
                    break;

					case 3: // 20.8 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x44);
						SX1276_setChannel(SX1276.Channel + 20830);
                    break;

					case 4: // 31.2 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x44);
						SX1276_setChannel(SX1276.Channel + 31250);
                    break;

					case 5: // 41.4 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x44);
						SX1276_setChannel(SX1276.Channel + 41670);
                    break;

					case 6: // 62.5 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x40);
                    break;

					case 7: // 125 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x40);
                    break;

					case 8: // 250 kHz
						SX1276_writeRegister(REG_LR_IFFREQ1, 0x40);
                    break;
                }
            }
            else
            {
                SX1276_writeRegister(REG_LR_DETECTOPTIMIZE, SX1276_readRegister(REG_LR_DETECTOPTIMIZE ) | 0x80);
            }

            rxContinuous = SX1276.LoRa.RxContinuous;

            if(SX1276.LoRa.FrequencyHop == true)
            {
                SX1276_writeRegister(REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT | //RFLR_IRQFLAGS_RXDONE | //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                		RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE );
						//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

                // DIO0 = RxDone
                // DIO2 = FhssChangeChannel

                SX1276_writeRegister(REG_DIOMAPPING1, (SX1276_readRegister(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
            }
            else
            {
                SX1276_writeRegister(REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT | //RFLR_IRQFLAGS_RXDONE | //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                				RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

                // DIO0 = RxDone

                SX1276_writeRegister(REG_DIOMAPPING1, (SX1276_readRegister(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);

            }

            SX1276_writeRegister(REG_LR_FIFORXBASEADDR, 0);
            SX1276_writeRegister(REG_LR_FIFOADDRPTR, 0);
        }
        break;
    }

    // memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    SX1276.State = RF_RX_RUNNING;

    if(SX1276.Modem == MODEM_FSK)
    {
    	// SX1276_setOpMode(RF_OPMODE_RECEIVER);
    }
    else
    {
        if(rxContinuous == true)
        {
            SX1276_setOpMode(RFLR_OPMODE_RECEIVER);
        }
        else
        {
            SX1276_setOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
        }
    }
}

void SX1276_setTx(void)
{
    switch(SX1276.Modem)
    {
    	case MODEM_FSK:
        {
            // DIO0 = PacketSent
            // DIO1 = FifoEmpty
            // DIO2 = FifoFull
            // DIO3 = FifoEmpty
            // DIO4 = LowBat
            // DIO5 = ModeReady

            SX1276_writeRegister(REG_DIOMAPPING1, (SX1276_readRegister(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) | RF_DIOMAPPING1_DIO1_01);

            SX1276_writeRegister(REG_DIOMAPPING2, (SX1276_readRegister(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK));

            SX1276.FskPacketHandler.FifoThresh = SX1276_readRegister(REG_FIFOTHRESH) & 0x3F;
        }
        break;

    	case MODEM_LORA:
        {
            if(SX1276.LoRa.FrequencyHop == true)
            {
                SX1276_writeRegister(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR |
                				RFLR_IRQFLAGS_VALIDHEADER | //RFLR_IRQFLAGS_TXDONE |
								RFLR_IRQFLAGS_CADDONE | //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
								RFLR_IRQFLAGS_CADDETECTED );

                // DIO0 = TxDone
                // DIO2 = FhssChangeChannel

                SX1276_writeRegister(REG_DIOMAPPING1, (SX1276_readRegister(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276_writeRegister(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR |
                				RFLR_IRQFLAGS_VALIDHEADER | //RFLR_IRQFLAGS_TXDONE |
								RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

                // DIO0 = TxDone

                SX1276_writeRegister(REG_DIOMAPPING1, (SX1276_readRegister(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
            }
        }
        break;
    }

    SX1276.State = RF_TX_RUNNING;

    SX1276_setOpMode(RF_OPMODE_TRANSMITTER);
}

/* Parameters value:
 *
 * modem        	Radio modem to be used [0: FSK, 1: LoRa]
 *
 * bandWidth    	Sets the bandwidth
 * 					FSK : >= 2600 and <= 250000 Hz
 *					LoRa: [0: 125 kHz, 1: 250 kHz,
 *					2: 500 kHz, 3: Reserved]
 *
 * dataRate     	Sets the Datarate
 * 					FSK : 600..300000 bits/s
 * 					LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *					10: 1024, 11: 2048, 12: 4096  chips]
 *
 * codeRate     	Sets the coding rate (LoRa only)
 *					FSK : N/A ( set to 0 )
 *					LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 *
 * bandwidthAfc 	Sets the AFC Bandwidth (FSK only)
 *					FSK : >= 2600 and <= 250000 Hz
 *					LoRa: N/A ( set to 0 )
 *
 * preambleLength  	Sets the Preamble length
 *					FSK : Number of bytes
 *					LoRa: Length in symbols (the hardware adds 4 more symbols)
 *
 * symbTimeout  	Sets the RxSingle timeout value
 *					FSK : timeout number of bytes
 *					LoRa: timeout in symbols
 *
 * fixLength       	Fixed length packets [0: variable, 1: fixed]
 *
 * payloadLength   	Sets payload length when fixed length is used
 *
 * crc	        	Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * frequencyHop    	Enables disables the intra-packet frequency hopping
 *					FSK : N/A ( set to 0 )
 *					LoRa: [0: OFF, 1: ON]
 *
 * hopPeriod    	Number of symbols between each hop
 *					FSK : N/A ( set to 0 )
 *					LoRa: Number of symbols
 *
 * iqInverted   	Inverts IQ signals (LoRa only)
 *					FSK : N/A ( set to 0 )
 *					LoRa: [0: not inverted, 1: inverted]
 *
 * rxContinuous 	Sets the reception in continuous mode
 *					[false: single mode, true: continuous mode]
 */
void SX1276_setRxConfig(uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLength,
                         uint16_t symbTimeout, bool fixLength,
                         uint8_t payloadLength,
                         bool crc, bool freqHop, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1276_setModem(SX1276.Modem);

    switch(SX1276.Modem)
    {
    	case MODEM_FSK:
        {
            SX1276.Fsk.BandWidth = bandwidth;
            SX1276.Fsk.DataRate = datarate;
            SX1276.Fsk.BandwidthAfc = bandwidthAfc;
            SX1276.Fsk.FixLength = fixLength;
            SX1276.Fsk.PayloadLength = payloadLength;
            SX1276.Fsk.Crc = crc;
            SX1276.Fsk.IqInverted = iqInverted;
            SX1276.Fsk.RxContinuous = rxContinuous;
            SX1276.Fsk.PreambleLength = preambleLength;
            SX1276.Fsk.RxSingleTimeout = (uint32_t)(symbTimeout * ((1.0 / (double)datarate) * 8.0) * 1000);

            datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);

            SX1276_writeRegister(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
            SX1276_writeRegister(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

            SX1276_writeRegister(REG_RXBW, SX1276_getFskBandwidthRegValue(bandwidth));
            SX1276_writeRegister(REG_AFCBW, SX1276_getFskBandwidthRegValue(bandwidthAfc));

            SX1276_writeRegister(REG_PREAMBLEMSB, (uint8_t)((preambleLength >> 8) & 0xFF));
            SX1276_writeRegister(REG_PREAMBLELSB, (uint8_t)(preambleLength & 0xFF));

            if(fixLength == 1)
            {
                SX1276_writeRegister(REG_PAYLOADLENGTH, payloadLength);
            }
            else
            {
                SX1276_writeRegister(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
            }

            SX1276_writeRegister(REG_PACKETCONFIG1, (SX1276_readRegister(REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK &
            				RF_PACKETCONFIG1_PACKETFORMAT_MASK) | ((fixLength == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) | (crc << 4));

            SX1276_writeRegister(REG_PACKETCONFIG2, (SX1276_readRegister(REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
        }
        break;

    	case MODEM_LORA:
        {
            if(bandwidth > 2)
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while(1);
            }

            bandwidth += 7;

            SX1276.LoRa.Bandwidth = bandwidth;
            SX1276.LoRa.Datarate = datarate;
            SX1276.LoRa.Coderate = coderate;
            SX1276.LoRa.PreambleLength = preambleLength;
            SX1276.LoRa.FixLength = fixLength;
            SX1276.LoRa.PayloadLength = payloadLength;
            SX1276.LoRa.Crc = crc;
            SX1276.LoRa.FrequencyHop = freqHop;
            SX1276.LoRa.HopPeriod = hopPeriod;
            SX1276.LoRa.IqInverted = iqInverted;
            SX1276.LoRa.RxContinuous = rxContinuous;

            if(datarate > 12)
            {
                datarate = 12;
            }
            else if(datarate < 6)
            {
                datarate = 6;
            }

            if(((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) ||
                ((bandwidth == 8) && (datarate == 12)))
            {
                SX1276.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.LoRa.LowDatarateOptimize = 0x00;
            }

            SX1276_writeRegister(REG_LR_MODEMCONFIG1, (SX1276_readRegister(REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
            				(bandwidth << 4) | (coderate << 1) | fixLength );

            SX1276_writeRegister(REG_LR_MODEMCONFIG2, (SX1276_readRegister(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
            				(datarate << 4) | (crc << 2) | ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

            SX1276_writeRegister(REG_LR_MODEMCONFIG3, (SX1276_readRegister(REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
            				(SX1276.LoRa.LowDatarateOptimize << 3));

            SX1276_writeRegister(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));

            SX1276_writeRegister(REG_LR_PREAMBLEMSB, (uint8_t)((preambleLength >> 8) & 0xFF));
            SX1276_writeRegister(REG_LR_PREAMBLELSB, (uint8_t)(preambleLength & 0xFF));

            if(fixLength == 1)
            {
                SX1276_writeRegister(REG_LR_PAYLOADLENGTH, payloadLength);
            }

            if(SX1276.LoRa.FrequencyHop == true)
            {
                SX1276_writeRegister(REG_LR_PLLHOP, (SX1276_readRegister(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                SX1276_writeRegister(REG_LR_HOPPERIOD, SX1276.LoRa.HopPeriod);
            }

            if((bandwidth == 9) && (SX1276.Channel > RF_MID_BAND_THRESH))
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276_writeRegister(REG_LR_HIGHBWOPTIMIZE1, 0x02);
                SX1276_writeRegister(REG_LR_HIGHBWOPTIMIZE2, 0x64);
            }
            else if(bandwidth == 9)
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276_writeRegister(REG_LR_HIGHBWOPTIMIZE1, 0x02);
                SX1276_writeRegister(REG_LR_HIGHBWOPTIMIZE2, 0x7F);
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276_writeRegister(REG_LR_HIGHBWOPTIMIZE1, 0x03);
            }

            if(datarate == 6)
            {
                SX1276_writeRegister(REG_LR_DETECTOPTIMIZE, (SX1276_readRegister(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                				RFLR_DETECTIONOPTIMIZE_SF6);

                SX1276_writeRegister(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
            }
            else
            {
                SX1276_writeRegister(REG_LR_DETECTOPTIMIZE, (SX1276_readData(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                				RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);

                SX1276_writeRegister(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
        }
        break;
    }
}

/* Parameters value:
 *
 * modem        			Radio modem to be used
 * 							[0: FSK, 1: LoRa]
 *
 * power        			Sets the output power [dBm]
 *
 * frequencyDeviation		Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 *
 * bandwWidth    			Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 *
 * dataRate     			Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 *
 * codeRate     			Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 *
 * preambleLength  			Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 *
 * fixLength       			Fixed length packets [0: variable, 1: fixed]
 *
 * crc        				Enables disables the CRC [0: OFF, 1: ON]
 *
 * frequencyHop    			Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 *
 * hopPeriod    			Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 *
 * iqInverted   			Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 *
 * timeout      			Transmission timeout [ms]
 *
 */
void SX1276_setTxConfig(SX1276_modemType modem, int8_t power, uint32_t frequencyDeviation,
                        uint32_t bandWidth, uint32_t dataRate,
                        uint8_t coderate, uint16_t preambleLength,
                        bool fixLength, bool crc, bool frequencyHop,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    SX1276_setModem(modem);

//  SX1276_setRfTxPower(power);

    switch(modem)
    {
    	case MODEM_FSK:
        {
            SX1276.Fsk.Power = power;
            SX1276.Fsk.FrequencyDeviation = frequencyDeviation;
            SX1276.Fsk.BandWidth = bandWidth;
            SX1276.Fsk.DataRate = dataRate;
            SX1276.Fsk.PreambleLength = preambleLength;
            SX1276.Fsk.FixLength = fixLength;
            SX1276.Fsk.Crc = crc;
            SX1276.Fsk.IqInverted = iqInverted;
            SX1276.Fsk.TxTimeout = timeout;

            frequencyDeviation = (uint16_t)((double)frequencyDeviation / (double)FREQ_STEP);

            SX1276_writeRegister(REG_FDEVMSB, (uint8_t)(frequencyDeviation >> 8)); // Set device frequency deviation
            SX1276_writeRegister(REG_FDEVLSB, (uint8_t)(frequencyDeviation & 0xFF));

            dataRate = (uint16_t)((double)XTAL_FREQ / (double)dataRate);

            SX1276_writeRegister(REG_BITRATEMSB, (uint8_t)(dataRate >> 8)); // Set device bit rate
            SX1276_writeRegister(REG_BITRATELSB, (uint8_t)(dataRate & 0xFF));

            SX1276_writeRegister(REG_PREAMBLEMSB, (preambleLength >> 8) & 0x00FF); // Set device pre-amble size
            SX1276_writeRegister(REG_PREAMBLELSB, preambleLength & 0xFF);

            SX1276_writeRegister(REG_PACKETCONFIG1, (SX1276_readRegister(REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
            				((fixLength == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) | (crc << 4)); // Set device Packet Config 1

            SX1276_writeRegister(REG_PACKETCONFIG2, (SX1276_readRegister(REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET)); // Set device Packet Config 2
        }
        break;

    	case MODEM_LORA:
        {
            SX1276.LoRa.Power = power;

            if(bandWidth > 2)
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }

            bandWidth += 7;

            SX1276.LoRa.Bandwidth = bandWidth;
            SX1276.LoRa.Datarate = dataRate;
            SX1276.LoRa.Coderate = coderate;
            SX1276.LoRa.PreambleLength = preambleLength;
            SX1276.LoRa.FixLength = fixLength;
            SX1276.LoRa.FrequencyHop = frequencyHop;
            SX1276.LoRa.HopPeriod = hopPeriod;
            SX1276.LoRa.Crc = crc;
            SX1276.LoRa.IqInverted = iqInverted;

            if(dataRate > 12)
            {
                dataRate = 12;
            }
            else if(dataRate < 6)
            {
                dataRate = 6;
            }

            if(((bandWidth == 7) && ((dataRate == 11) || (dataRate == 12))) ||
                ((bandWidth == 8) && (dataRate == 12)))
            {
                SX1276.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.LoRa.LowDatarateOptimize = 0x00;
            }

            if(SX1276.LoRa.FrequencyHop == true)
            {
                SX1276_writeRegister(REG_LR_PLLHOP, (SX1276_readRegister(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                SX1276_writeRegister(REG_LR_HOPPERIOD, SX1276.LoRa.HopPeriod);
            }

            SX1276_writeRegister(REG_LR_MODEMCONFIG1, (SX1276_readRegister(REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
            				(bandWidth << 4) | (coderate << 1) | fixLength);

            SX1276_writeRegister(REG_LR_MODEMCONFIG2, (SX1276_readRegister(REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) |
            				(dataRate << 4) | (crc << 2));

            SX1276_writeRegister(REG_LR_MODEMCONFIG3, (SX1276_readRegister(REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
            				(SX1276.LoRa.LowDatarateOptimize << 3));

            SX1276_writeRegister(REG_LR_PREAMBLEMSB, (preambleLength >> 8) & 0x00FF);
            SX1276_writeRegister(REG_LR_PREAMBLELSB, preambleLength & 0xFF);

            if(dataRate == 6)
            {
                SX1276_writeRegister(REG_LR_DETECTOPTIMIZE, (SX1276_readRegister(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                				RFLR_DETECTIONOPTIMIZE_SF6);

                SX1276_writeRegister(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
            }
            else
            {
                SX1276_writeRegister(REG_LR_DETECTOPTIMIZE, (SX1276_readRegister(REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) |
                				RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);

                SX1276_writeRegister(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
        }
        break;
    }
}

void SX1276_setTxContinuousWave(uint32_t freq, int8_t power, uint16_t time)
{
    uint32_t timeout = (uint32_t)time * 1000;

    SX1276_setChannel(freq);

    SX1276_setTxConfig(MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout);

    SX1276_writeRegister(REG_PACKETCONFIG2, (SX1276_readRegister(REG_PACKETCONFIG2) & RF_PACKETCONFIG2_DATAMODE_MASK));

    // Disable radio interrupts
    SX1276_writeRegister(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11);
    SX1276_writeRegister(REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10);

    SX1276.State = RF_TX_RUNNING;

    SX1276_setOpMode(RF_OPMODE_TRANSMITTER);
}


void SX1276_send(uint8_t *data, uint8_t size)
{
	switch(SX1276.Modem)
    {
    	case MODEM_FSK:
        {
            SX1276.FskPacketHandler.NbBytes = 0;
            SX1276.FskPacketHandler.Size = size;

            if(SX1276.Fsk.FixLength == false)
            {
            	SX1276_writeMoreThanOneData(REG_FIFO, (uint8_t*)&size, 1);
            }
            else
            {
            	SX1276_writeRegister(REG_PAYLOADLENGTH, size);
            }

            if((size > 0) && (size <= 64))
            {
                SX1276.FskPacketHandler.ChunkSize = size;
            }
            else
            {
//                memcpy1(RxTxBuffer, buffer, size );

//                SX1276.FskPacketHandler.ChunkSize = 32;
            }

            SX1276_writeMoreThanOneData(REG_FIFO, data, SX1276.FskPacketHandler.ChunkSize);

            SX1276.FskPacketHandler.NbBytes += SX1276.FskPacketHandler.ChunkSize;
        }
        break;

    	case MODEM_LORA:
        {
            if(SX1276.LoRa.IqInverted == true)
            {
            	SX1276_writeRegister(REG_LR_INVERTIQ, (SX1276_readRegister(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON);
            	SX1276_writeRegister(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            }
            else
            {
            	SX1276_writeRegister(REG_LR_INVERTIQ, (SX1276_readRegister(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF);
				SX1276_writeRegister(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            SX1276.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1276_writeRegister(REG_LR_PAYLOADLENGTH, size);

            // Full buffer used for Tx
            SX1276_writeRegister(REG_LR_FIFOTXBASEADDR, 0);
            SX1276_writeRegister(REG_LR_FIFOADDRPTR, 0);

            // Write payload buffer
            SX1276_writeMoreThanOneData(REG_FIFO, data, size);
        }
        break;
    }
}

void SX1276_startCad(void)
{
    switch(SX1276.Modem)
    {
    	case MODEM_FSK:
        {

        }
        break;

    	case MODEM_LORA:
        {
            SX1276_writeRegister(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR |
            				RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE | //RFLR_IRQFLAGS_CADDONE |
							RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // | //RFLR_IRQFLAGS_CADDETECTED
            				);

            // DIO3 = CADDone
            SX1276_writeRegister(REG_DIOMAPPING1, (SX1276_readRegister(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO3_MASK) | RFLR_DIOMAPPING1_DIO3_00);

            SX1276.State = RF_CAD;
            SX1276_setOpMode(RFLR_OPMODE_CAD);
        }
        break;

    	default:

    	break;
    }
}

void SX1276_init()
{
	PORTA->PCR[12] |= (1 << 8) | (1 << 4) | (1 << 1); // DIO G1 Set alt1 mode for GPIO, passive filter on
	PORTA->PCR[16] |= (1 << 8) | (1 << 4) | (1 << 1); // DIO G2 Set alt1 mode for GPIO, passive filter on
	PORTA->PCR[17] |= (1 << 8) | (1 << 4) | (1 << 1); // DIO G3 Set alt1 mode for GPIO, passive filter on

	PORTA->PCR[12] |= (1 << 19) | (1 << 16);  // interrupt configuration on PORTA pin 12, on rising edge
	PORTA->PCR[16] |= (1 << 19) | (1 << 16);  // interrupt configuration on PORTA pin 4, on rising edge
	PORTA->PCR[17] |= (1 << 19) | (1 << 17);  // interrupt configuration on PORTA pin 5, on rising edge

	PORTD->PCR[4] |= (1 << 8) | (1 << 4) | (1 << 1); // DIO G0 Set alt1 mode for GPIO, passive filter on
	PORTD->PCR[5] |= (1 << 8) | (1 << 4) | (1 << 1); // DIO G3 Set alt1 mode for GPIO, passive filter on
	PORTD->PCR[7] |= (1 << 8) | (1 << 4) | (1 << 1); // DIO G4 Set alt1 mode for GPIO, passive filter on
	PORTD->PCR[6] |= (1 << 8) | (1 << 4) | (1 << 1); // DIO G5 Set alt1 mode for GPIO, passive filter on

	PORTD->PCR[4] |= (1 << 19) | (1 << 16);  // interrupt configuration on PORTD pin 4, on rising edge
	PORTD->PCR[7] |= (1 << 19) | (1 << 16);  // interrupt configuration on PORTD pin 7, on rising edge
	PORTD->PCR[6] |= (1 << 19) | (1 << 16);  // interrupt configuration on PORTD pin 6, on rising edge

	NVIC->IP[7] |= (1 << 23); // preemptive priority set as 2; min value is 3, max and default is 0
	NVIC->ISER[0] |= (1 << 30); // PORTA interrupt enable in NVIC

	NVIC->IP[7] |= (1 << 30); // preemptive priority set as 2; min value is 3, max and default is 0
	NVIC->ISER[0] |= (1 << 31); // PORTD interrupt enable in NVIC


	SIM->SCGC4 |= (1 << 22); // Clock on SPI0 Module

	PORTD->PCR[1] = (1 << 9); // SPI SCK (alt2 mode)
	PORTD->PCR[2] = (1 << 9); // SPI MOSI (alt2 mode)
	PORTD->PCR[3] = (1 << 9); // SPI MISO (alt2 mode)
	PORTD->PCR[0] = (1 << 9); // SPI CS (alt2 mode)

	SPI0->C1 |= (1 << 6) | (1 << 3) | (1 << 4) | (1 << 3); // SPI0 Enable in Master Mode, clock polarity: active low (idle high)

	SPI0->BR |= (1 << 2) | (1 << 1); // Prescaler divisor set as 1, Baud Rate divisor set as 16, so Freq. = bus clock / (16 * ...) = ... [Hz]

	SX1276_setOpMode(RF_OPMODE_TRANSMITTER);

	SX1276_setTxConfig(MODEM_FSK, 1, 20, 125000, 50000, 0, 4, 0, 0, 0, 0, 0, 0); // Initial modem configuration in FSK

	SX1276_setTx();

	SX1276_setOpMode(RF_OPMODE_TRANSMITTER);
}

void PORTA_IRQHandler(void)
{
	if((GPIOA->PDIR & (1 << 12)) != (1 << 12)) // G1 routine - Dio1
	{
		switch(SX1276.State)
		{
			case RF_RX_RUNNING:

			switch(SX1276.Modem)
			{
				case MODEM_FSK:

					// FifoLevel interrupt

					// Read received packet size

					if((SX1276.FskPacketHandler.Size == 0) && (SX1276.FskPacketHandler.NbBytes == 0))
		            {
						if(SX1276.Fsk.FixLength == false)
						{
							SX1276_readMoreThanOneData(REG_FIFO, (uint8_t*)&SX1276.FskPacketHandler.Size, 1);
						}
						else
						{
							SX1276.FskPacketHandler.Size = SX1276_readRegister(REG_PAYLOADLENGTH);
		                }
		            }

					// ERRATA 3.1 - PayloadReady Set for 31.25ns if FIFO is Empty
					//
					//              When FifoLevel interrupt is used to offload the
					//              FIFO, the microcontroller should  monitor  both
					//              PayloadReady  and FifoLevel interrupts, and
					//              read only (FifoThreshold-1) bytes off the FIFO
					//              when FifoLevel fires

					if((SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes) >= SX1276.FskPacketHandler.FifoThresh)
					{
						SX1276_readMoreThanOneData(REG_FIFO, (RxTxBuffer + SX1276.FskPacketHandler.NbBytes), SX1276.FskPacketHandler.FifoThresh - 1 );
						SX1276.FskPacketHandler.NbBytes += SX1276.FskPacketHandler.FifoThresh - 1;
					}
					else
					{
						SX1276_readMoreThanOneData(REG_FIFO, (RxTxBuffer + SX1276.FskPacketHandler.NbBytes), SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes);
						SX1276.FskPacketHandler.NbBytes += (SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes);
					}

					break;

				case MODEM_LORA:

					// Clear Irq
					SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);

					SX1276.State = RF_IDLE;

					break;

				default:
		                break;
			}
			break;

			case RF_TX_RUNNING:
				switch(SX1276.Modem)
				{
		            case MODEM_FSK:
		            	// FifoEmpty interrupt
		            	if((SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes) > SX1276.FskPacketHandler.ChunkSize)
		                {
		                    SX1276_writeMoreThanOneData(REG_FIFO, (RxTxBuffer + SX1276.FskPacketHandler.NbBytes), SX1276.FskPacketHandler.ChunkSize);
		                    SX1276.FskPacketHandler.NbBytes += SX1276.FskPacketHandler.ChunkSize;
		                }
		                else
		                {
		                    // Write the last chunk of data
		                    SX1276_writeMoreThanOneData(REG_FIFO, RxTxBuffer + SX1276.FskPacketHandler.NbBytes, SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes);
		                    SX1276.FskPacketHandler.NbBytes += SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes;
		                }
		                break;

		            case MODEM_LORA:

		            	break;

		            default:
		                break;
				}

				break;

				default:
					break;
		    }
	}

	if((GPIOA->PDIR & (1 << 16)) != (1 << 16)) // G2 routine - Dio2
	{
		 switch(SX1276.State)
		 {
		 	 case RF_RX_RUNNING:

		 		 switch(SX1276.Modem)
		 		 {
		            case MODEM_FSK:
		                // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.

//		                if(SX1276.DIO4.port == NULL)
//		                {
//		                    SX1276.FskPacketHandler.PreambleDetected = true;
//		                }

		                if((SX1276.FskPacketHandler.PreambleDetected != 0) && (SX1276.FskPacketHandler.SyncWordDetected == 0))
		                {
		                    SX1276.FskPacketHandler.SyncWordDetected = true;

		                    SX1276.FskPacketHandler.RssiValue = -(SX1276_readRegister(REG_RSSIVALUE) >> 1);

		                    SX1276.FskPacketHandler.AfcValue = (int32_t)((double)(((uint16_t)SX1276_readRegister(REG_AFCMSB) << 8) |
		                                                                             (uint16_t)SX1276_readRegister(REG_AFCLSB)) *
		                                                                             (double)FREQ_STEP);

		                    SX1276.FskPacketHandler.RxGain = (SX1276_readRegister(REG_LNA) >> 5) & 0x07;
		                }
		                break;

		            case MODEM_LORA:

		            	if(SX1276.LoRa.FrequencyHop == true)
		                {
		                    // Clear Irq
		                    SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);
		                }
		                break;

		            default:

		            	break;
		            }

		            break;

		 	case RF_TX_RUNNING:

		 		switch(SX1276.Modem)
		 		{
		 			case MODEM_FSK:

		 			break;

		 			case MODEM_LORA:

		 			if(SX1276.LoRa.FrequencyHop == true)
		 			{
		 				// Clear Irq
		 				SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);
		 			}
		 			break;

		 			default:

		 			break;
		 		}

		 		break;

		 		default:

		 		break;
		    }
	}

	if((GPIOA->PDIR & (1 << 17)) != (1 << 17)) // G3 routine - Dio3
	{
		 switch(SX1276.Modem)
		 {
		    case MODEM_FSK:

		    	break;

		    case MODEM_LORA:
		    {
		    	if((SX1276_readRegister(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) == RFLR_IRQFLAGS_CADDETECTED)
		        {
		            // Clear Irq
		            SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
		        }
		        else
		        {
		            // Clear Irq
		            SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
		        }
		        break;
		    }
		    default:

		    	break;
		 }
	}

	PORTA->PCR[12] |= (1 << 24); // clear port A interrupt flag
	PORTA->PCR[16] |= (1 << 24); // clear port A interrupt flag
	PORTA->PCR[17] |= (1 << 24); // clear port A interrupt flag
}


void PORTD_IRQHandler(void)
{
	if((GPIOD->PDIR & (1 << 4)) != (1 << 4)) // G0 routine - Dio0
	{
		volatile uint8_t irqFlags = 0;

		switch(SX1276.State)
		{
			case RF_RX_RUNNING:

			switch(SX1276.Modem)
			{
				case MODEM_FSK:

				if(SX1276.Fsk.Crc == true)
				{
					irqFlags = SX1276_readRegister(REG_IRQFLAGS2);

					if((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK)
					{
						// Clear Irqs
						SX1276_writeRegister(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | RF_IRQFLAGS1_PREAMBLEDETECT | RF_IRQFLAGS1_SYNCADDRESSMATCH);

						SX1276_writeRegister(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

						if(SX1276.Fsk.RxContinuous == false)
						{
							SX1276.State = RF_IDLE;
						}
						else
						{
							// Continuous mode restart Rx chain
							SX1276_writeRegister(REG_RXCONFIG, SX1276_readRegister(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
						}

						SX1276.FskPacketHandler.PreambleDetected = false;
						SX1276.FskPacketHandler.SyncWordDetected = false;
						SX1276.FskPacketHandler.NbBytes = 0;
						SX1276.FskPacketHandler.Size = 0;

						break;
					}
				}

				// Read received packet size
				if((SX1276.FskPacketHandler.Size == 0) && (SX1276.FskPacketHandler.NbBytes == 0))
				{
					if(SX1276.Fsk.FixLength == false)
					{
						SX1276_readMoreThanOneData(REG_FIFO, (uint8_t*)&SX1276.FskPacketHandler.Size, 1);
					}
					else
					{
						SX1276.FskPacketHandler.Size = SX1276_readRegister(REG_PAYLOADLENGTH);
					}

					SX1276_readMoreThanOneData(REG_FIFO, RxTxBuffer + SX1276.FskPacketHandler.NbBytes, SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes);

					SX1276.FskPacketHandler.NbBytes += (SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes);
				}
				else
				{
					SX1276_readMoreThanOneData(REG_FIFO, RxTxBuffer + SX1276.FskPacketHandler.NbBytes, SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes);

					SX1276.FskPacketHandler.NbBytes += (SX1276.FskPacketHandler.Size - SX1276.FskPacketHandler.NbBytes);
				}

				if(SX1276.Fsk.RxContinuous == false)
				{
					SX1276.State = RF_IDLE;
				}
				else
				{
					// Continuous mode restart Rx chain
					SX1276_writeRegister(REG_RXCONFIG, SX1276_readRegister(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
				}

				SX1276.FskPacketHandler.PreambleDetected = false;
				SX1276.FskPacketHandler.SyncWordDetected = false;
				SX1276.FskPacketHandler.NbBytes = 0;
				SX1276.FskPacketHandler.Size = 0;

				break;


				case MODEM_LORA:
				{
					// Clear Irq
					SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

					irqFlags = SX1276_readRegister(REG_LR_IRQFLAGS);

					if((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
					{
						// Clear Irq
						SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

						if(SX1276.LoRa.RxContinuous == false)
						{
							SX1276.State = RF_IDLE;
						}

						break;
					}

					// Returns SNR value [dB] rounded to the nearest integer value
					SX1276.LoRaPacketHandler.SnrValue = (((int8_t )SX1276_readRegister(REG_LR_PKTSNRVALUE)) + 2) >> 2;

					int16_t rssi = SX1276_readRegister(REG_LR_PKTRSSIVALUE);

					if(SX1276.LoRaPacketHandler.SnrValue < 0)
					{
						if(SX1276.Channel > RF_MID_BAND_THRESH)
						{
							SX1276.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4) +
							SX1276.LoRaPacketHandler.SnrValue;
						}
						else
						{
							SX1276.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4) +
							SX1276.LoRaPacketHandler.SnrValue;
						}
					}
					else
					{
						if(SX1276.Channel > RF_MID_BAND_THRESH)
						{
							SX1276.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4);
						}
						else
						{
							SX1276.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4);
						}
					}

					SX1276.LoRaPacketHandler.Size = SX1276_readRegister(REG_LR_RXNBBYTES);

					SX1276_writeRegister(REG_LR_FIFOADDRPTR, SX1276_readRegister(REG_LR_FIFORXCURRENTADDR));

					SX1276_readMoreThanOneData(REG_FIFO, RxTxBuffer, SX1276.LoRaPacketHandler.Size);

					if(SX1276.LoRa.RxContinuous == false)
					{
						SX1276.State = RF_IDLE;
					}
				}
				break;

				default:

				break;
			}

			break;

			case RF_TX_RUNNING:

				// TxDone interrupt
				switch(SX1276.Modem)
				{
					case MODEM_LORA:

						// Clear Irq
		                SX1276_writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
		                // Intentional fall through

					case MODEM_FSK:

		            default:

		            SX1276.State = RF_IDLE;

		            break;
				}
				break;

			default:

			break;
		}
	}

	if((GPIOD->PDIR & (1 << 7)) != (1 << 7)) // G4 routine - Dio4
	{
		switch(SX1276.Modem)
		{
		    case MODEM_FSK:
		    {
		    	if(SX1276.FskPacketHandler.PreambleDetected == false)
		    	{
		    		SX1276.FskPacketHandler.PreambleDetected = true;
		    	}
		    }
		    break;

		    case MODEM_LORA:

		    	break;

		    default:
		        break;
		}
	}

	if((GPIOD->PDIR & (1 << 6)) != (1 << 6)) // G5 routine - Dio5
	{

	}

	PORTD->PCR[4] |= (1 << 24); // clear port D interrupt flag
	PORTD->PCR[7] |= (1 << 24); // clear port D interrupt flag
	PORTD->PCR[6] |= (1 << 24); // clear port D interrupt flag
}
