/**
 * @file max31865.h
 * @author Shahid Hashmi (shahidh@acevin.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "headers.h"
#include <stdint.h>
#include "spi.h"
#include "rtd_cli.h"

/* General */
#define MIN_OPERATING_TEMP  -40
#define MAX_OPERATING_TEMP  125
#define ADC_BIT_RESOLUTION  32767
#define MAX_SPI_FREQ    1000000
#define NUMBER_OF_BYTES 2

#define MAX_RTD_TEMP_LIMIT  500
#define MIN_RTD_TEMP_LIMIT  -300

/* Resistance To Temperature */
#define RTD_ALPHA   0.00390830 //3.90830*pow(10,-3)
#define RTD_BETA    -0.0000005775 //-5.77500*pow(10,-7)
#define RTD_GAMMA   -0.00000000000418301 //-4.18301*pow(10,-12) //for -200C - 0C & 0C - 850C
#define RTD_RES_ZERO_DEGREE 100 //RTD Resistance at 0 Degree
#define RTD_REF    200 //Considering Rref as 400
#define RTD_COUNT_TO_RESISTANCE(x)      ((float)(x * RTD_REF)/ADC_BIT_RESOLUTION)

/* Registers */
#define READ_CONFIGURATION_ADDR     0X00 //Read Configuration
#define WRITE_CONFIGURATION_ADDR    0X80 //Write Configuration
#define READ_RTD_MSB    0X01 //Read RTD MSBs First
#define READ_RTD_LSB    0x02 //Read RTD LSBs First
#define READ_HFT_MSB    0x03 //Read High Fault Threshold(HFT) MSB
#define WRITE_HFT_MSB   0x83 //Write High Fault Threshold(HFT) MSB
#define READ_HFT_LSB    0x04 //Read High Fault Threshold(HFT) LSB
#define WRITE_HFT_LSB   0x84 //Write High Fault Threshold(HFT) LSB
#define READ_LFT_MSB    0x05 //Read Low Fault Threshold(LFT) MSB
#define WRITE_LFT_MSB   0x85 //Write Low Fault Threshold(LFT) MSB
#define READ_LFT_LSB    0x06 //Read Low Fault Threshold(LFT) LSB
#define WRITE_LFT_LSB   0x86 //Write Low Fault Threshold(LFT) LSB
#define READ_FAULT_STATUS   0x07 //Read Fault Status


/* Functions */
spi_t *max_spi_init(char *spidev, char spiMode , int freq);
uint8_t max_spi_deinit(spi_t *spi);
uint8_t Max_init_Configuration(void);
uint8_t Max_single_byte_read(spi_t *spi, uint8_t address, uint8_t *pRxData);
uint8_t Max_single_byte_write(spi_t *spi, uint8_t address, uint8_t data);
uint8_t Max_mutiple_write_byte(spi_t *spi, uint8_t address, uint8_t *pWriteData, uint8_t length);
uint8_t Max_mutiple_read_byte(spi_t *spi, uint8_t address, uint8_t *pData, uint8_t length);
uint8_t Max_fault_detection(spi_t *spi);
uint8_t Max_sensor_scan(spi_t *spi, st_rtd_sample_t *pScanMaxRead, int Channel);
void Max_Temp_read(spi_t *spi, st_rtd_sample_t *pMaxRead, uint8_t Channel);
float Max_Temp_cal(uint16_t Max_ADC);