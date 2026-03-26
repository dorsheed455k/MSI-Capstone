#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdint.h>

#include "usbd_cdc.h"

int _write(int file, char *ptr, int len)
{
    /* Use a timeout or loop until we are no longer busy */
    uint8_t result;
    do {
        result = CDC_Transmit_FS((uint8_t*)ptr, len);
    } while (result == USBD_BUSY);
    
    return len;
}
