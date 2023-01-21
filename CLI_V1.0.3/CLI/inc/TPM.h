/**
 * @file TPM.h
 * @author Shahid Hashmi (shahidh@acevin.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Library */
#include <stdint.h>
#include "headers.h"

/* General */
#define TPM_MIN_ARG_REQ     2


/* Functions */
uint8_t TPM_init(char *filename, uint16_t deviceAddr);
void TPM_deinit(void);
void TPM_help(void);
void TPM_cmd(int argc, char **argv, void *addtionalargc);
