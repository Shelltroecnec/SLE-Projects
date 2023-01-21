/**
 * @file TPM.c
 * @author Shahid Hashmi (shahidh@acevin.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Library */
#include "headers.h"
#include "TPM.h"

/* This Function is Used to initilization of TPM in i2c Mode */
uint8_t TPM_init(char *filename, uint8_t deviceAddr)
{
}

/* This Function is Used to deinitize TPM in i2c */
void TPM_deinit(void)
{
    i2c_close(stI2C);
    i2c_free(stI2C);
}

/* This Function is used to display the TPM help parameters/options */
void TPM_help(void)
{
    CLOG("TPM Command Usage :\n");
    CLOG("    %s -c TPM [Function] [VALUE]\n", exec_filename);
    CLOG("       \nAvailable TPM Functions :-\n");
    CLOG("     1 - TPM Start\n");
    CLOG("     2 - TPM Self Test\n");
    CLOG("     3 - TPM Create Endrosement Key\n");
    CLOG("     4 - TPM Take Owership\n");
    CLOG("     5 - TPM Create Wrap Key\n");
    CLOG("     6 - TPM Load Key\n");
    CLOG("     7 - TPM Seal\n");
    CLOG("     8 - TPM Un-Seal\n");
    CLOG("     9 - TPM Sign\n");
    CLOG("     10 - TPM Verify Sign\n");
    CLOG("     11 - TPM Get Public Key\n");
    CLOG("     12 - TPM Reset\n");
    CLOG("     13 - TPM ForceClear\n");
    CLOG("     14 - TPM Enable/Activate\n");
    CLOG("     15 - TPM Disable/Deactivate\n");
    CLOG("     16 - TPM Display Known Keys\n");
}

/* TPM command is used to Parse the CLI argument for the TPM */
void TPM_cmd(int argc, char **argv, void *addtionalargc)
{

    if (argc < TPM_MIN_ARG_REQ) {
        WLOG("[TPM] Less number of arguments provided for TPM Cmd!!\n");
        TPM_help();
        return;
    }

    int TPMSelection = 0;
    //Parsing of the arguments
    TPMSelection = atoi(argv[1]);
    int Value = atoi(argv[2]);
    
    CLOG("[TPM] TPM Function selected = %d\n", TPMSelection);
    // DLOG("[IOEX] Port = %d\n", port);
    // DLOG("[IOEX] Pin = %x\n", pin);
    // DLOG("[IOEX] dir = %s\n", dir);
    CLOG("[TPM] Value = %d\n", Value);

    if (TPMSelection < 1 && TPMSelection > 16) {
        WLOG("[TPM] Invalid TPM Function!!!\n");
        TPM_help();
        return;
    }

    //Apply it on IO expander registers
    //ioexpander_write_read(config.IOExpDev, IOEXP_INDEX_NUMBER(IOExpSelection), dir, port, pin, (char)value);
}

