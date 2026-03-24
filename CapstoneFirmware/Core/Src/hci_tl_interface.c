#include "hci_tl.h"
#include "bluenrg_m0.h"
#include "main.h"
#include "hci.h"

extern BlueNRG_M0_Handle_t ble;

/* ST Middleware depends on these functions being implemented */

void hci_tl_lowlevel_init(void)
{
    /* This is called by hci_init(). 
       SPI and GPIO are already initialized in main.c */
}

/* Local functions for the IO bus */
static int32_t TL_BLE_Init(void* pConf)
{
    (void)pConf;
    return 0;
}

static int32_t TL_BLE_Receive(uint8_t* pData, uint16_t len)
{
    uint8_t header_tx[5] = {0x0B, 0x00, 0x00, 0x00, 0x00};
    uint8_t header_rx[5];
    uint16_t read_len;

    if (HAL_GPIO_ReadPin(BT_IRQ_GPIO_Port, BT_IRQ_Pin) == GPIO_PIN_RESET) {
        return 0;
    }

    /* Manually handle CS to keep it low for the whole transaction */
    HAL_GPIO_WritePin(ble.cs_port, ble.cs_pin, GPIO_PIN_RESET);

    /* 5-byte header exchange */
    if (HAL_SPI_TransmitReceive(ble.hspi, header_tx, header_rx, 5, 100) != HAL_OK) {
        HAL_GPIO_WritePin(ble.cs_port, ble.cs_pin, GPIO_PIN_SET);
        return -1;
    }

    read_len = header_rx[3] | (header_rx[4] << 8);

    if (read_len > 0) {
        if (read_len > len) read_len = len;
        
        /* Read data bytes */
        if (HAL_SPI_Receive(ble.hspi, pData, read_len, 100) != HAL_OK) {
            HAL_GPIO_WritePin(ble.cs_port, ble.cs_pin, GPIO_PIN_SET);
            return -1;
        }
    }

    HAL_GPIO_WritePin(ble.cs_port, ble.cs_pin, GPIO_PIN_SET);
    return read_len;
}

static int32_t TL_BLE_Send(uint8_t* pData, uint16_t len)
{
    if (BlueNRG_M0_SpiWrite(&ble, pData, len) == BLUENRG_OK) {
        return len;
    }
    return -1;
}

static int32_t TL_BLE_GetTick(void)
{
    return (int32_t)HAL_GetTick();
}

static int32_t TL_BLE_Reset(void)
{
    BlueNRG_M0_ResetPulse(&ble, 10, 100);
    return 0;
}

void BLE_Transport_Register(void)
{
    tHciIO fops;
    fops.Init = TL_BLE_Init;
    fops.Receive = TL_BLE_Receive;
    fops.Send = TL_BLE_Send;
    fops.GetTick = TL_BLE_GetTick;
    fops.Reset = TL_BLE_Reset;
    
    hci_register_io_bus(&fops);
}
