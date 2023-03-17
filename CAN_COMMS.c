#include <stdio.h>
#include "include/CAN_COMMS.h"

#include "string.h"

#define BAUDRATE    115200

// Tamaño de buffer correspondiente a las estructuras de envio y recepcion a la motor control board
#define SIZE_BUFFER_TX  8
#define SIZE_BUFFER_RX  13

#define START_CODE_HEADER   0xABCD
#define ID_MOTOR_MODULE     0xF0
#define ID_IMU_MODULE       0xFA

uart_port_t PORT_UART;

QueueHandle_t   uartQueue;

rx_motor_control_board_t motorControlBoard;

void canInit(uint8_t txPin,uint8_t rxPin,uart_port_t portUart){

    PORT_UART = portUart;
    uart_config_t configCan = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(uart_param_config(portUart , &configCan));
    ESP_ERROR_CHECK(uart_set_pin(portUart, txPin, rxPin, -1, -1));
    uart_driver_install(portUart, SIZE_BUFFER_RX, SIZE_BUFFER_TX, 10, &uartQueue, 0);       // TODO: ajustar sizes, tamaño de cola y el flag de interrupcion
}

void sendMotorData(int16_t motR,int16_t motL,uint8_t enable,uint8_t ordenCode){

    tx_motor_control_board_t motor_control ={

        .start = START_CODE_HEADER,
	    .idModule = 1,
        .enable = true,
        .motR = motR,
        .motL = motL,
	    .ordenCode =0,
        .checksum = (START_CODE_HEADER)^(1)^(true)^(100)^(100)^(0),
    };

    uart_write_bytes(PORT_UART,&motor_control,sizeof(motor_control));
}

void readMotorData(){
    rx_motor_control_board_t rxData;
    uint16_t newChecksum = 0,lentghRx = 0;

    ESP_ERROR_CHECK(uart_get_buffered_data_len(PORT_UART, (size_t*)&lentghRx));

    if(lentghRx >= sizeof(rx_motor_control_board_t)){
        uart_read_bytes(PORT_UART, &rxData, lentghRx, 100);

        if( rxData.start == START_CODE_HEADER && rxData.idModule == ID_MOTOR_MODULE){

            newChecksum = (uint16_t)(rxData.start ^ rxData.idModule ^ rxData.speedL ^ rxData.speedR  ^ rxData.batVoltage ^ rxData.tempUc ^ rxData.ordenCode ^ rxData.errorCode ^ rxData.status );
            
            if( newChecksum == rxData.checksum){
                memcpy(&motorControlBoard,&rxData,sizeof(rx_motor_control_board_t));
                printf("Data nueva copiada: %d",motorControlBoard.status);
            }
        }
    }
    uart_flush(PORT_UART);                                  // limpio lo remanente
}