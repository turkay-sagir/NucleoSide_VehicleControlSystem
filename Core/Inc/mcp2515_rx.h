#ifndef MCP2515_RX_H
#define MCP2515_RX_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==== MCP2515 SPI Komutları ==== */
#define MCP_RESET        0xC0
#define MCP_READ         0x03
#define MCP_WRITE        0x02
#define MCP_READ_RX      0x90
#define MCP_READ_STATUS  0xA0
#define MCP_LOAD_TX      0x40
#define MCP_RTS_TX0      0x81
#define MCP_BIT_MODIFY   0x05

/* ==== MCP2515 Register Adresleri ==== */
#define MCP_CANCTRL      0x0F
#define MCP_CANSTAT      0x0E
#define MCP_CNF1         0x2A
#define MCP_CNF2         0x29
#define MCP_CNF3         0x28
#define MCP_CANINTE      0x2B
#define MCP_CANINTF      0x2C
#define MCP_RXB0CTRL     0x60
#define MCP_RXB0SIDH     0x61

/* ==== Mod Tanımları ==== */
#define MCP_MODE_NORMAL  0x00
#define MCP_MODE_CONFIG  0x80

/* MCP2515 INT pin (Interrupt) */
#define MCP2515_INT_PORT    GPIOG
#define MCP2515_INT_PIN     GPIO_PIN_14

/* ==== SPI Arabirim Tanımları ==== */
#define MCP2515_CS_PORT_RX     GPIOD
#define MCP2515_CS_PIN_RX      GPIO_PIN_14
extern SPI_HandleTypeDef hspi1;

/* ==== CAN Mesaj Yapısı ==== */
typedef struct {
    uint16_t id;
    uint8_t data[8];
    uint8_t length;
} CAN_Message_RX;

/* ==== Fonksiyon Prototipleri ==== */
bool MCP2515_RX_Init(void);
bool MCP2515_RX_ReceiveMessage(CAN_Message_RX *msg);
void MCP2515_RX_Reset(void);
void MCP2515_RX_WriteRegister(uint8_t address, uint8_t data);
uint8_t MCP2515_RX_ReadRegister(uint8_t address);

#endif /* MCP2515_RX_H */
