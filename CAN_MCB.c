#include "include/CAN_MCB.h"
#include "string.h"
#include "../../../include/main.h"              // TODO: mirar este def

extern QueueSetHandle_t queueMotorControl; 
config_init_mcb_t mcbConfigInit;
rx_motor_control_board_t motorControlBoard;

static void sendMotorData(int16_t motR,int16_t motL,uint8_t enable);

static void controlHandler(void *pvParameters) {
    output_motors_t newVel;

    queueMotorControl = xQueueCreate(1,sizeof(output_motors_t));

    while(true) {
        if (xQueueReceive(queueMotorControl,&newVel,pdMS_TO_TICKS(1))) {
            sendMotorData(newVel.motorR,newVel.motorL,newVel.enable);
        }
        // vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void mcbInit(config_init_mcb_t *config) {

    mcbConfigInit = *config;
    /*configuro periferico*/
    uart_config_t uart_config={
        .baud_rate = MAINBOARD_BAUDATE,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .stop_bits = UART_STOP_BITS_1, 
    };
    ESP_ERROR_CHECK(uart_param_config(config->numUart,&uart_config));

    /* configuro pines a utilizar por el uart*/
    ESP_ERROR_CHECK(uart_set_pin(config->numUart,config->txPin,config->rxPin,-1,-1));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(config->numUart, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // Write data to UART.
    // char* test_str = "This is a test string.\n";
    // uart_write_bytes(config->numUart,(const char*)test_str, strlen(test_str));
    printf("\nMCB INIT\n");

    xTaskCreate(controlHandler,"MCB handler task",4096,NULL,5,NULL);
}


typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;


void sendMotorData(int16_t motR,int16_t motL,uint8_t enable){

    // uint16_t calcChecksum = (START_CODE_HEADER)^(ID_MOTOR_MODULE)^(cont)^(motL)^(enable)^(0x00);
    // tx_motor_control_board_t motor_control ={
    //     .start = START_CODE_HEADER,
	//     .idModule = ID_MOTOR_MODULE,
    //     .motR = cont,
    //     .motL = motL,
    //     .enable = enable,
    //     .ordenCode = 0x00,
    //     .checksum = calcChecksum
    // };
    // printf("Previo a envio uart: checksum calc: %x\n",calcChecksum);
    // uart_write_bytes(mcbConfigInit.numUart,&motor_control,sizeof(motor_control));
    // printf("Posterior a envio uart\n");

    uint16_t calcChecksum = (START_CODE_HEADER)^(motL)^(motR);
    SerialCommand command = {
        .start = START_CODE_HEADER,
        .speed = motL,
        .steer = motR,
        .checksum = calcChecksum
    };
    if (uart_write_bytes(mcbConfigInit.numUart,&command,sizeof(command)) >=0 ) {
        printf("Send MCB OK\n");
    }
    else {
        printf("Send MCB ERROR\n");
    }
    printf("speedL: %d, speedR(steer): %d\n",command.speed,command.steer);
}

void readMotorData(){
    rx_motor_control_board_t rxData;
    uint16_t newChecksum = 0,lentghRx = 0;

    ESP_ERROR_CHECK(uart_get_buffered_data_len(mcbConfigInit.numUart, (size_t*)&lentghRx));

    if(lentghRx >= sizeof(rx_motor_control_board_t)){
        uart_read_bytes(mcbConfigInit.numUart, &rxData, lentghRx, 100);

        if( rxData.start == START_CODE_HEADER && rxData.idModule == ID_MOTOR_MODULE){

            newChecksum = (uint16_t)(rxData.start ^ rxData.idModule ^ rxData.speedL ^ rxData.speedR  ^ rxData.batVoltage ^ rxData.tempUc ^ rxData.ordenCode ^ rxData.errorCode ^ rxData.status );
            
            if( newChecksum == rxData.checksum){
                memcpy(&motorControlBoard,&rxData,sizeof(rx_motor_control_board_t));
                printf("Data nueva copiada: %d",motorControlBoard.status);
            }
        }
    }
    uart_flush(mcbConfigInit.numUart);                                  // limpio lo remanente
}