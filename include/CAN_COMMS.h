#include "driver/gpio.h"
#include "driver/uart.h"


enum{
    ENABLE_MOTORS,
    DISABLE_MOTORS,
    POWER_OFF,
    POWER_ON

} ordenCode;

typedef struct{
    uint16_t    start;
    uint8_t     idModule;
    int16_t     motR;
    int16_t     motL;
    uint8_t     enable;
    uint8_t     ordenCode;
    uint16_t    checksum;
}tx_motor_control_board_t;


typedef struct{
	uint16_t    start;
	uint8_t     idModule;
    int16_t     speedR;
    int16_t     speedL;
    uint16_t    batVoltage;
    uint16_t    tempUc;
    uint16_t    status;
    uint8_t     ordenCode;
    uint8_t     errorCode;
    uint32_t    checksum;
}
rx_motor_control_board_t;



void canInit(uint8_t txPin,uint8_t rxPin,uart_port_t portUart);
void sendMotorData(int16_t motR,int16_t motL,uint8_t enable,uint8_t ordenCode);
// void readMotorData();