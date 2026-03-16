#ifndef BLUENRG_CONF_H
#define BLUENRG_CONF_H

#include "main.h"
#include <string.h>

/* Memory function mappings expected by BlueNRG stack */
#define BLUENRG_MEMSET              memset
#define BLUENRG_MEMCPY              memcpy

/* Some BlueNRG source files use lowercase variants */
#define BLUENRG_memset              memset
#define BLUENRG_memcpy              memcpy

/* HCI payload/event sizing */
#define HCI_READ_PACKET_SIZE        128
#define HCI_WRITE_PACKET_SIZE       128
#define HCI_MAX_EVENT_SIZE          260
#define HCI_MAX_PAYLOAD_SIZE        255

/* Default timeout for HCI command/response waits (ms) */
#define HCI_DEFAULT_TIMEOUT_MS      1000

#endif /* BLUENRG_CONF_H */
