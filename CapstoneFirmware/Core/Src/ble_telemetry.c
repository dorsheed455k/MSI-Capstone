#include "ble_telemetry.h"
#include "hci_tl.h"
#include "hci.h"
#include "bluenrg_aci_const.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include <string.h>
#include <stddef.h>
#include <stdint.h>

static uint16_t telemetry_service_handle;
static uint16_t telemetry_char_handle;
static uint16_t dev_name_char_handle;
static uint16_t appearance_char_handle;

/* helper macro */
#define COPY_UUID_128(uuid_struct, b15,b14,b13,b12,b11,b10,b9,b8,b7,b6,b5,b4,b3,b2,b1,b0) \
  do { \
    uuid_struct[0]  = b0;  uuid_struct[1]  = b1;  uuid_struct[2]  = b2;  uuid_struct[3]  = b3; \
    uuid_struct[4]  = b4;  uuid_struct[5]  = b5;  uuid_struct[6]  = b6;  uuid_struct[7]  = b7; \
    uuid_struct[8]  = b8;  uuid_struct[9]  = b9;  uuid_struct[10] = b10; uuid_struct[11] = b11; \
    uuid_struct[12] = b12; uuid_struct[13] = b13; uuid_struct[14] = b14; uuid_struct[15] = b15; \
  } while(0)

void BLE_Stack_Init(void)
{
	 uint8_t ret;
	 uint16_t service_handle, dev_name_handle, appearance_handle;

     /* 1. Register custom transport layer functions */
     extern void BLE_Transport_Register(void);
     BLE_Transport_Register();

     /* 2. Initialize the HCI library and reset BlueNRG module */
     hci_init(NULL, NULL);

     /* 3. Continue with GATT/GAP initialization */
	 ret = aci_gatt_init();
	 (void)ret;

	 ret = aci_gap_init_IDB05A1(0x01,
	                               0,
	                               20,
	                               &service_handle,
	                               &dev_name_handle,
	                               &appearance_handle);
	  (void)ret;

	  dev_name_char_handle = dev_name_handle;
	  appearance_char_handle = appearance_handle;

	  const char name[] = "MSI_Drive";
	  aci_gatt_update_char_value(service_handle,
	                               dev_name_char_handle,
	                               0,
	                               sizeof(name) - 1,
	                               (uint8_t*)name);
}

void BLE_Telemetry_Init(void)
{
    uint8_t ret;
    uint8_t service_uuid[16];
    uint8_t char_uuid[16];

    /* custom service UUID */
    COPY_UUID_128(service_uuid,
                  0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,
                  0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0);

    /* custom characteristic UUID */
    COPY_UUID_128(char_uuid,
                  0x12,0x34,0x56,0x78,0x12,0x34,0x56,0x78,
                  0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF1);

    ret = aci_gatt_add_serv(UUID_TYPE_128,
                            service_uuid,
                            PRIMARY_SERVICE,
                            8,
                            &telemetry_service_handle);
    (void)ret;

    ret = aci_gatt_add_char(telemetry_service_handle,
                            UUID_TYPE_128,
                            char_uuid,
                            sizeof(TelemetryPacket_t),
                            CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                            ATTR_PERMISSION_NONE,
                            GATT_NOTIFY_ATTRIBUTE_WRITE,
                            16,
                            1,
                            &telemetry_char_handle);
    (void)ret;
}

void BLE_StartAdvertising(void)
{
    const char local_name[] = "MSI_Drive";

    aci_gap_set_discoverable(ADV_IND,
                             0x00A0,   /* min interval */
                             0x00F0,   /* max interval */
                             PUBLIC_ADDR,
                             NO_WHITE_LIST_USE,
                             sizeof(local_name) - 1,
                             local_name,
                             0,
                             NULL,
                             0,
                             0);
}

void BLE_SendTelemetry(const TelemetryPacket_t *pkt)
{
    if (!pkt) return;

    aci_gatt_update_char_value(telemetry_service_handle,
                               telemetry_char_handle,
                               0,
                               sizeof(TelemetryPacket_t),
                               (uint8_t*)pkt);
}
