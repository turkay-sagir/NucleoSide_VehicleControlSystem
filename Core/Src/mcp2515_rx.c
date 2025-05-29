#include "mcp2515_rx.h"
#include "main.h"
#include <string.h>

/* Dahili yardımcı fonksiyonlar */
static void MCP2515_RX_Select(void)
{
    HAL_GPIO_WritePin(MCP2515_CS_PORT_RX, MCP2515_CS_PIN_RX, GPIO_PIN_RESET);
}

static void MCP2515_RX_Unselect(void)
{
    HAL_GPIO_WritePin(MCP2515_CS_PORT_RX, MCP2515_CS_PIN_RX, GPIO_PIN_SET);
}

void MCP2515_RX_Reset(void)
{
    MCP2515_RX_Select();
    uint8_t cmd = MCP_RESET;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    MCP2515_RX_Unselect();
    HAL_Delay(50);
}

void MCP2515_RX_WriteRegister(uint8_t address, uint8_t data)
{
    uint8_t tx[] = {MCP_WRITE, address, data};
    MCP2515_RX_Select();
    HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), HAL_MAX_DELAY);
    MCP2515_RX_Unselect();
}

uint8_t MCP2515_RX_ReadRegister(uint8_t address)
{
    uint8_t tx[2] = {MCP_READ, address};
    uint8_t rx[2] = {0};

    MCP2515_RX_Select();
    HAL_StatusTypeDef result1 = HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_StatusTypeDef result2 = HAL_SPI_Receive(&hspi1, &rx[1], 1, HAL_MAX_DELAY);
    MCP2515_RX_Unselect();

    if (result1 != HAL_OK || result2 != HAL_OK) {
        printf("SPI iletişim hatası! (TX:%d, RX:%d)\r\n", result1, result2);
    }

    return rx[1];
}

static void MCP2515_RX_BitModify(uint8_t address, uint8_t mask, uint8_t data)
{
    uint8_t tx[4] = {MCP_BIT_MODIFY, address, mask, data};
    MCP2515_RX_Select();
    HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
    MCP2515_RX_Unselect();
}

static uint8_t MCP2515_RX_ReadStatus(void)
{
    uint8_t cmd = MCP_READ_STATUS;
    uint8_t status = 0;

    MCP2515_RX_Select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    MCP2515_RX_Unselect();

    return status;
}

/* ==== MCP2515 ALICI MODÜLÜ BAŞLAT ==== */
bool MCP2515_RX_Init(void)
{
    MCP2515_RX_Reset();

    // Konfigürasyon moduna geç
    MCP2515_RX_WriteRegister(MCP_CANCTRL, MCP_MODE_CONFIG);
    uint8_t mode;
    uint32_t timeout = HAL_GetTick() + 100;

    do {
        mode = MCP2515_RX_ReadRegister(MCP_CANSTAT);
        HAL_Delay(1);
    } while ((mode & 0xE0) != MCP_MODE_CONFIG && HAL_GetTick() < timeout);

    if ((mode & 0xE0) != MCP_MODE_CONFIG) {
        printf("MCP2515 CONFIG moda geçemedi! CANSTAT: 0x%02X\r\n", mode);
        return false;
    }

    // Baudrate: 125kbps @ 8MHz
//    MCP2515_RX_WriteRegister(MCP_CNF1, 0x03);
//    MCP2515_RX_WriteRegister(MCP_CNF2, 0xF0);
//    MCP2515_RX_WriteRegister(MCP_CNF3, 0x86);

    MCP2515_RX_WriteRegister(MCP_CNF1, 0x05);  // BRP = 6
    MCP2515_RX_WriteRegister(MCP_CNF2, 0xB8);  // Prop = 1, PS1 = 6
    MCP2515_RX_WriteRegister(MCP_CNF3, 0x05);  // PS2 = 6

    // RXB0 buffer'ı her mesajı kabul edecek şekilde ayarla (filtre kapalı)
    MCP2515_RX_WriteRegister(MCP_RXB0CTRL, 0x00);



    // Normal moda geç
    MCP2515_RX_WriteRegister(MCP_CANCTRL, MCP_MODE_NORMAL);


    // RX0 interrupt aktif et
    MCP2515_RX_WriteRegister(MCP_CANINTE, 0x01);

    // MCP2515'in NORMAL moda geçmesini bekle

    do {
        mode = MCP2515_RX_ReadRegister(MCP_CANSTAT);
        HAL_Delay(1);
    } while ((mode & 0xE0) != MCP_MODE_NORMAL && HAL_GetTick() < timeout);

    if ((mode & 0xE0) != MCP_MODE_NORMAL) {
        printf("MCP2515 NORMAL moda geçemedi! CANSTAT: 0x%02X\r\n", mode);
        return false;
    }

    printf("MCP2515 NORMAL moda geçti!\r\n");
    return true;
}

/* ==== MCP2515 Üzerinden Mesaj Alma ==== */
bool MCP2515_RX_ReceiveMessage(CAN_Message_RX *msg)
{
    uint8_t status = MCP2515_RX_ReadStatus();

    if (status & 0x01)  // RX0IF set mi?
    {
        uint8_t buffer[13];

        MCP2515_RX_Select();
        uint8_t cmd = MCP_READ_RX | 0x00; // RXB0 için
        HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(&hspi1, buffer, sizeof(buffer), HAL_MAX_DELAY);
        MCP2515_RX_Unselect();

        // 11-bit CAN ID oluştur
        msg->id = ((uint16_t)buffer[0] << 3) | (buffer[1] >> 5);
        msg->length = buffer[4] & 0x0F; // DLC (0-8)

        memcpy(msg->data, &buffer[5], msg->length);

        // RX flag temizle
        MCP2515_RX_BitModify(MCP_CANINTF, 0x01, 0x00);

        return true;
    }

    return false;
}


void MCP2515_SimpleCheck(void)
{
    uint8_t tx[] = {MCP_READ, MCP_CANSTAT};
    uint8_t rx = 0;

    MCP2515_RX_Select();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
    MCP2515_RX_Unselect();

    printf("CANSTAT register direct SPI read: 0x%02X\r\n", rx);
}

