/**
 * @file max31865.c
 * @author Shahid Hashmi (shahidh@acevin.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "max31865.h"
#include "headers.h"
#include "spi.h"
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <stdbool.h>
#include <sys/ioctl.h>

/* This Function is Used to initilize the MAX31865 Configuration Register */
uint8_t Max_init_Configuration(void)
{
    spi_t *conf=NULL;
    uint8_t tx_buff[NUMBER_OF_BYTES];
    uint8_t rx_buff[NUMBER_OF_BYTES];

    memset(rx_buff,0, NUMBER_OF_BYTES);
    
    if((conf = max_spi_init(config.rtd_spi_device, SPI_MODE_1, SPI_FREQ_1M))==NULL)
        WLOG("MAX31865 SPI INITILIZATION FAILED !\r\n");

    tx_buff[0] = WRITE_CONFIGURATION_ADDR & 0xFF;
    tx_buff[1] |= 0xD0; //VBIAS=1, Conversion mode =1(Auto), 0=Shot Conversion(Auto-Clear), 1=3-Wire, Fault Detection=00, Fault Status = Clear, 60Hz
    
    ILOG("[MAX] SPI Transfer = %x %x",tx_buff[0],tx_buff[1]);
    ILOG("tx_buff[0] = %x",tx_buff[0]);
    ILOG("tx_buff[1] = %x",tx_buff[1]);

    if((spi_transfer(conf,tx_buff,rx_buff,NUMBER_OF_BYTES))!=0)
        ILOG("Error in SPI MAX Transfer\r\n");
     
    ILOG("[MAX] RTD Configuration SPI Sucessfull !\r\n");
    
    return 0;
}

/* This Function is used to initilization of MAX31865 with SPI */
spi_t *max_spi_init(char *spi_dev, char spiMode, int freq)
{
    spi_t *spi = NULL;
    spi = spi_new();
    CLOG("Created new SPI\n");
    spi->dev = spi_dev;

    CLOG("MAX31865 Initilization: SPI Dev: %s, mode: %d, freq: %d\n", spi_dev, spiMode, freq);
    if (spi_open(spi, spi_dev, spiMode, freq))
        ILOG("Print error Code Here\r\n");
    
    Max_single_byte_write(spi, WRITE_CONFIGURATION_ADDR, 0xD0); //VBIAS=1, Conversion mode =1(Auto), 0=Shot Conversion(Auto-Clear), 1=3-Wire, Fault Detection=00, Fault Status = Clear, 60Hz
    usleep(100000);

    return spi;
}

/* This Function to de-initilization of MAX31865 SPI */
uint8_t max_spi_deinit(spi_t *spi)
{
    CLOG("Closing SPI Dev %s\n",spi->dev);
    if (spi_close(spi)) {
        ILOG("Fail to close the MAX31865 SPI device\r\n");
        return -1;
    }
    return 0;
}


/* This Function is Used to Read a byte from MAX31865 */
uint8_t Max_single_byte_read(spi_t *spi, uint8_t address, uint8_t *pRxData)
{
    uint8_t tx_buff[NUMBER_OF_BYTES] = {address, 0};
    
    //if((address != READ_CONFIGURATION_ADDR) | (address != READ_RTD_MSB) | (address != READ_RTD_LSB) | (address != READ_HFT_MSB) |
    //(address != READ_HFT_LSB) | (address != READ_LFT_MSB) | (address != READ_LFT_LSB) | (address != READ_FAULT_STATUS))
    //    WLOG("Invalid Register Address !\r\n");

    //tx_buff[0] =  address & 0xFF; //<--
    
    CLOG("[MAX] SPI Transfer = %x",tx_buff[0]);
    CLOG("tx_buff[0] = %x",tx_buff[0]);

    if(spi_transfer(spi,tx_buff,pRxData,NUMBER_OF_BYTES)){
    WLOG("Error in SPI Transfer !: %s\r\n",spi->error.errmsg);
    return spi->error.c_errno;
    }

    CLOG("Recieved Data = %x\r\n",pRxData);
    
    return 0;
}

/* This Function is Used for Single Byte Write into MAX38165 Register */
uint8_t Max_single_byte_write(spi_t *spi, uint8_t address, uint8_t data)
{

    uint8_t tx_buff[NUMBER_OF_BYTES];
    uint8_t rx_buff[NUMBER_OF_BYTES];
    memset(tx_buff,0, NUMBER_OF_BYTES);
    memset(rx_buff,0, NUMBER_OF_BYTES);
        
    //if((address != WRITE_CONFIGURATION_ADDR) | (address != WRITE_HFT_MSB) | (address != WRITE_HFT_LSB) | 
    //(address != WRITE_LFT_MSB) | (address != WRITE_LFT_LSB))
    //    WLOG("Invalid Register Address !\r\n");

    tx_buff[0] =  address;
    tx_buff[1] |= data;
    CLOG("[MAX] SPI Single Byte Transfer = %x %x",tx_buff[0],tx_buff[1]);
    CLOG("tx_buff[0] = %x",tx_buff[0]);
    CLOG("tx_buff[1] = %x",tx_buff[1]);

    if(spi_transfer(spi,tx_buff,rx_buff,NUMBER_OF_BYTES)){
    WLOG("Error in SPI Transfer !: %s\r\n",spi->error.errmsg);
    return spi->error.c_errno;
    }

    CLOG("[MAX] Successful Transfer of Write Byte\r\n");
    return 0;
}

/* This Function is Used to Tranfer Multiple Bytes to MAX31865 */
uint8_t Max_mutiple_write_byte(spi_t *spi, uint8_t address, uint8_t *pWriteData, uint8_t length)
{
    uint8_t tx_buff[255];
    uint8_t rx_buff[255];
    uint8_t i;

    memset(rx_buff,0, length);
    
    //if((address != WRITE_CONFIGURATION_ADDR) | (address != WRITE_HFT_MSB) | (address != WRITE_HFT_LSB) | 
    //(address != WRITE_LFT_MSB) | (address != WRITE_LFT_LSB))
    //    CLOG("Invalid Register Address !\r\n");

    tx_buff[0] =  address & 0xFF;
    for(i=1; i<length; i++)
        tx_buff[i] |= *pWriteData++;
    
    CLOG("[MAX] SPI Mutiple Write Transfer = %x ",tx_buff);
    CLOG("tx_buff[0], Address = %x",tx_buff[0]);

    if(spi_transfer(spi, tx_buff, rx_buff, length+1)){
    WLOG("Error in SPI Transfer !: %s\r\n",spi->error.errmsg);
    return spi->error.c_errno;
    }
    
    CLOG("[MAX] Successful Transfer of Mutiple bytes = %d",length);
    return 0;
}

/* This is function is Used for Mutiple Bytes Read */
uint8_t Max_mutiple_read_byte(spi_t *spi, uint8_t address, uint8_t *pData, uint8_t length)
{
    uint8_t tx_buff[255];
    memset(tx_buff, 0, length);
    
    //if((address != READ_CONFIGURATION_ADDR) | (address != READ_RTD_MSB) | (address != READ_RTD_LSB) | (address != READ_HFT_MSB) |
    //(address != READ_HFT_LSB) | (address != READ_LFT_MSB) | (address != READ_LFT_LSB) | (address != READ_FAULT_STATUS))
    //    CLOG("Invalid Register Address !\r\n");

    tx_buff[0] =  address & 0xFF;
    
    CLOG("[MAX] SPI Mutiple Transfer = %x",tx_buff);
    
    if(spi_transfer(spi, tx_buff, pData, length+1)){
    WLOG("Error in SPI Transfer !: %s\r\n",spi->error.errmsg);
    return spi->error.c_errno;
    }

    CLOG("[MAX] Recieved Multiple Data = %x\r\n",*pData);
    
    return 0;
}

/* This Function is Used to Read Fault Status Register */
uint8_t Max_fault_detection(spi_t *spi)
{
    uint8_t pData[2];
    memset(pData,0,2);
    uint8_t faultValue=0;

    Max_single_byte_read(spi, READ_FAULT_STATUS, pData );
    
    faultValue = pData[1];
    ILOG("Fault = %x\r\n",faultValue);

    Max_single_byte_write(spi,WRITE_CONFIGURATION_ADDR,0xD2); //Clearing bit 1 of Configuration Reg, For Clearing Fault Status
    Max_single_byte_write(spi,WRITE_CONFIGURATION_ADDR,0xD0); //Setting Default Value of Condiguration Register
    usleep(100000);

    //Verify if the fault bit is reset
    Max_single_byte_read(spi, READ_RTD_LSB, pData); //reading the fault status register to get fault specifications
    faultValue = pData[1];
    DLOG("After clearing fault status, RTD LSB: %x\n",faultValue);
    Max_single_byte_write(spi, WRITE_CONFIGURATION_ADDR, 0x00); //Clearing the Configuration Register

    return faultValue;
}

/* This Function is Used to Read the Temperature from the RTD */
void Max_Temp_read(spi_t *spi, st_rtd_sample_t *pMaxRead, uint8_t Channel)
{
    uint8_t RTDRx_buff[4];
    memset(RTDRx_buff,0,4);
    uint8_t RTDValueMSB =0, RTDValueLSB = 0;

    if(Max_mutiple_read_byte(spi,READ_RTD_MSB, RTDRx_buff,NUMBER_OF_BYTES)== 0){
    RTDValueMSB = RTDRx_buff[1];
    RTDValueLSB = RTDRx_buff[2];
    pMaxRead->rtd_fault = RTDValueLSB & 0x01;
    //Printing the Readed Data from the RTD Register
    CLOG("RTD MSB = %x, RTD LSB = %x\r\n",RTDValueMSB,RTDValueLSB);

    if(pMaxRead->rtd_fault){
    CLOG("RTD Faults has been detected !");
    return;
    }
    pMaxRead->adc_count[Channel-1][pMaxRead->counter] = (uint16_t)((RTDValueMSB << 7)+(RTDValueLSB >> 1)); //Calculating ADC corrspomding to Channel
    }
}

/* This Function is Used to Calculate the Temperature */
float Max_Temp_cal(uint16_t Max_ADC)
{
    float Temperature = 0.0f, Resistance = 0.0f;
    float nr, dr;

    //Resistance = (float)(Max_ADC * Rref)/ADC_BIT_RESOLUTION; //Calculating ADC to Resistance -> Rrtd = (ADC * Rref) / 2^15
    Resistance = RTD_COUNT_TO_RESISTANCE(Max_ADC);

    //Calculating Resistance to Temperature
    if (Resistance >= RTD_RES_ZERO_DEGREE) {
        Temperature = RTD_RES_ZERO_DEGREE * RTD_ALPHA + sqrt(RTD_RES_ZERO_DEGREE * RTD_RES_ZERO_DEGREE * RTD_ALPHA * RTD_ALPHA - 4 * RTD_RES_ZERO_DEGREE * RTD_BETA * (RTD_RES_ZERO_DEGREE - Resistance));
        Temperature = Temperature / (2 * RTD_RES_ZERO_DEGREE * RTD_BETA);
    }

    else {
        Temperature = ((Resistance / RTD_RES_ZERO_DEGREE) - 1);
        Temperature = Temperature / (RTD_ALPHA + (100 * RTD_BETA));

        DLOG("RDT: Temperature = %0.3f\n", Temperature);

        nr = 1 + (RTD_ALPHA * Temperature) + (RTD_BETA * Temperature * Temperature) + 
                ((RTD_GAMMA * Temperature * Temperature * Temperature) * (Temperature - 100) -
                (Resistance / RTD_RES_ZERO_DEGREE));
        dr = RTD_ALPHA + (2 * RTD_BETA * Temperature) - (300 * RTD_GAMMA * Temperature * Temperature) +
             (4 * RTD_GAMMA * Temperature * Temperature * Temperature);
        Temperature = Temperature - (nr / dr);

        nr = 1 + (RTD_ALPHA * Temperature) + (RTD_BETA * Temperature * Temperature) +
             ((RTD_GAMMA * Temperature * Temperature * Temperature) * (Temperature - 100) -
              (Resistance / RTD_RES_ZERO_DEGREE));
        dr = RTD_ALPHA + (2 * RTD_BETA * Temperature) - (300 * RTD_GAMMA * Temperature * Temperature) +
             (4 * RTD_GAMMA * Temperature * Temperature * Temperature);
        Temperature = Temperature - (nr / dr);

        nr = 1 + (RTD_ALPHA * Temperature) + (RTD_BETA * Temperature * Temperature) +
             ((RTD_GAMMA * Temperature * Temperature * Temperature) * (Temperature - 100) -
              (Resistance / RTD_RES_ZERO_DEGREE));
        dr = RTD_ALPHA + (2 * RTD_BETA * Temperature) - (300 * RTD_GAMMA * Temperature * Temperature) +
             (4 * RTD_GAMMA * Temperature * Temperature * Temperature);
        Temperature = Temperature - (nr / dr);
    }
    return Temperature;
}

/* This Function is Use to RTD Scan */
uint8_t Max_sensor_scan(spi_t *spi, st_rtd_sample_t *pScanMaxRead, int Channel)
{
    uint8_t SampleCount =0;
    Max_single_byte_write(spi,WRITE_CONFIGURATION_ADDR,0xD2); //If Fault bit is SET it Clears
    pScanMaxRead->counter=0;

    for(SampleCount = 0; SampleCount < pScanMaxRead->rtd_sample_cnt; SampleCount++)
    {
        usleep(100000); //100ms delay     //Sampling Time delay to get a new conversion in RTD data register
        Max_Temp_read(spi,pScanMaxRead,Channel);
        pScanMaxRead->counter +=1;

        if(pScanMaxRead->rtd_fault){
        ILOG("Falut Bit SET\r\n");
        return 1;
        }
    }

    Max_single_byte_write(spi,WRITE_CONFIGURATION_ADDR, 0x00); //Reseting the Configuration Resgister

    pScanMaxRead->avg_adc_count[Channel - 1] = avg_samples_integer((uint16_t*)&pScanMaxRead->adc_count[Channel-1][0], pScanMaxRead->rtd_sample_cnt, &pScanMaxRead->max_adc_count[Channel - 1], &pScanMaxRead->min_adc_count[Channel - 1]);
    pScanMaxRead->rtd_avg_temp[Channel - 1] = Max_Temp_cal(pScanMaxRead->avg_adc_count[Channel - 1]);
    pScanMaxRead->rtd_max_temp[Channel - 1] = Max_Temp_cal(pScanMaxRead->max_adc_count[Channel - 1]);
    pScanMaxRead->rtd_min_temp[Channel - 1] = Max_Temp_cal(pScanMaxRead->min_adc_count[Channel - 1]);

    return 0;
}
