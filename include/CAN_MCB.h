#ifndef CAN_MCB_H
#define CAN_MCB_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"

#define MAINBOARD_BAUDATE   115200

#define START_CODE_HEADER   0xABCD
#define ID_MOTOR_MODULE     0xF0

// MotorControlBoard = MCB

typedef struct {
    gpio_num_t txPin;
    gpio_num_t rxPin;
    uart_port_t numUart;
}config_init_mcb_t;

typedef struct {
    uint16_t    start;
    uint16_t    idModule;       // Se trabaja con 2 bytes por problemas de alineacion si es solo 1 byte
    int16_t     motR;
    int16_t     motL;
    uint8_t     enable;
    uint8_t     ordenCode;
    uint16_t    checksum;
} tx_motor_control_board_t;

typedef struct {
	uint16_t    start;
	uint16_t     idModule;
    int16_t     speedR;
    int16_t     speedL;
    uint16_t    batVoltage;
    uint16_t    tempUc;
    uint16_t    status;
    uint8_t     ordenCode;
    uint8_t     errorCode;
    uint16_t    checksum;
} rx_motor_control_board_t;

void mcbInit(config_init_mcb_t *config);
void mcbReadData();

#endif