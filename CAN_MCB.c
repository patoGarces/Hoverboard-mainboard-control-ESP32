#include "include/CAN_MCB.h"
#include "string.h"
#include "../../../include/main.h"              // TODO: mirar este def

extern QueueSetHandle_t queueMotorControl; 
config_init_mcb_t mcbConfigInit;
rx_motor_control_board_t motorControlBoard;

static QueueHandle_t spp_uart_queue;

static void sendMotorData(int16_t motR,int16_t motL,uint8_t enable);

typedef struct {
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;


void sendMotorData(int16_t motR,int16_t motL,uint8_t enable) {

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
    // printf("envio uart\n");

    uint16_t calcChecksum = (START_CODE_HEADER)^(motL)^(motR);
    SerialCommand command = {
        .start = START_CODE_HEADER,
        .speed = motL,
        .steer = motR,
        .checksum = calcChecksum
    };

    // uart_write_bytes(mcbConfigInit.numUart,&command,sizeof(command));

    uart_write_bytes(UART_NUM_2,&command, sizeof(command));
    // printf("speedL: %d, speedR: %d\n",command.speed,command.steer);
}

typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksum;
} SerialFeedback;

static void processMCBData(uint8_t *data,uint8_t length) {
    // rx_motor_control_board_t rxData;
    // uint16_t newChecksum = 0,lentghRx = 0;

    // ESP_ERROR_CHECK(uart_get_buffered_data_len(mcbConfigInit.numUart, (size_t*)&lentghRx));

    // if(lentghRx >= sizeof(rx_motor_control_board_t)){
    //     uart_read_bytes(mcbConfigInit.numUart, &rxData, lentghRx, 100);

    //     if( rxData.start == START_CODE_HEADER && rxData.idModule == ID_MOTOR_MODULE){

    //         newChecksum = (uint16_t)(rxData.start ^ rxData.idModule ^ rxData.speedL ^ rxData.speedR  ^ rxData.batVoltage ^ rxData.tempUc ^ rxData.ordenCode ^ rxData.errorCode ^ rxData.status );
            
    //         if( newChecksum == rxData.checksum){
    //             memcpy(&motorControlBoard,&rxData,sizeof(rx_motor_control_board_t));
    //             printf("Data nueva copiada: %d",motorControlBoard.status);
    //         }
    //     }
    // }
    // uart_flush(mcbConfigInit.numUart);                                  // limpio lo remanente

    SerialFeedback MCBPacket;
    // uint8_t rxData[100];
    // uint16_t newChecksum = 0,lentghRx = 0;

    // printf("Leo buffer uart rx\n");
    // ESP_ERROR_CHECK(uart_get_buffered_data_len(mcbConfigInit.numUart, (size_t*)&lentghRx));
    // printf("FIN buffer uart rx\n");
    
    // printf("processing data length: %d\n",length);
    if (length == sizeof(MCBPacket)) {
        // printf("TAMAÑO CORRECTO DEL PAQUETE!\n");
        memcpy(&MCBPacket,&data,sizeof(SerialFeedback));

        printf("Data leida: %x, %x, %x, %x, %x, %x, %x, %x, %x\n",MCBPacket.start,MCBPacket.cmd1,MCBPacket.cmd2,MCBPacket.speedR_meas,MCBPacket.speedL_meas,MCBPacket.batVoltage,MCBPacket.boardTemp,MCBPacket.cmdLed,MCBPacket.checksum);
    }
    // 

    // if(lentghRx >= sizeof(SerialFeedback)){
    //     // uart_read_bytes(mcbConfigInit.numUart, &rxData, lentghRx, 1000);

    //     if( rxData.start == START_CODE_HEADER) {

    //         newChecksum = (uint16_t)(rxData.start ^ rxData.cmd1 ^ rxData.cmd2 ^ rxData.speedR_meas ^ rxData.speedL_meas 
    //                                        ^ rxData.batVoltage ^ rxData.boardTemp ^ rxData.cmdLed);
    //         if( newChecksum == rxData.checksum){
    //             // memcpy(&motorControlBoard,&rxData,sizeof(SerialFeedback));
    //             // printf("Data nueva copiada: %d",motorControlBoard.status);

    //             printf("Nuevo paquete OK -> voltage: %d\tspeedL: %d\tspeedR:%d\n",rxData.batVoltage,rxData.speedL_meas,rxData.speedR_meas);
    //         }
    //         else {
    //             printf("error checksum\n");
    //         }
    //     }
    //     uart_flush(mcbConfigInit.numUart);                                  // limpio lo remanente
    // }
}


static void controlHandler(void *pvParameters) {
    output_motors_t newVel;
    uart_event_t event;
    uint8_t data[100];

    while(true) {
        if (xQueueReceive(queueMotorControl,&newVel,0)) {
            sendMotorData(newVel.motorR,newVel.motorL,newVel.enable);
        }

        if (xQueueReceive(spp_uart_queue, (void * )&event, 0)) {
            
            if(event.type == UART_DATA) {
                if (event.size) {
                    // printf("Data recibida: %d\n",event.size);
                    uint8_t length = uart_read_bytes(UART_NUM_2, data, event.size, 0);
                    if (length > 0) {
                        // Procesar los datos recibidos
                        processMCBData(data, length);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mcbInit(config_init_mcb_t *config) {

    mcbConfigInit = *config;
    /*configuro periferico*/
    // uart_config_t uart_config={
        // .baud_rate = MAINBOARD_BAUDRATE,
    //     .data_bits = UART_DATA_8_BITS,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .parity = UART_PARITY_DISABLE,
    //     .rx_flow_ctrl_thresh = 122,
    //     .stop_bits = UART_STOP_BITS_1, 
    //     .source_clk = UART_SCLK_APB,
    // };

    // // Setup UART buffered IO with event queue
    // const int uart_buffer_size = (1024 * 2);
    // // Install UART driver using an event queue here
    // ESP_ERROR_CHECK(uart_driver_install(config->numUart, uart_buffer_size, uart_buffer_size, 10, &spp_uart_queue, 0));
    // ESP_ERROR_CHECK(uart_param_config(config->numUart,&uart_config));
    // /* configuro pines a utilizar por el uart*/
    // ESP_ERROR_CHECK(uart_set_pin(config->numUart,config->txPin,config->rxPin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));




    uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_2, 4096, 8192, 10,&spp_uart_queue,0);
    //Set UART parameters
    uart_param_config(UART_NUM_2, &uart_config);
    // Set UART pins
    // uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2,GPIO_CAN_TX,GPIO_CAN_RX,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));        // <-- PROBADO FUNCIONANDO
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2,config->txPin,config->rxPin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));

    // Write data to UART.
    // char* test_str = "This is a test string.\n";
    // uart_write_bytes(config->numUart,(const char*)test_str, strlen(test_str));

    printf("\nMCB INIT\n");

    xTaskCreate(controlHandler,"MCB handler task",4096,NULL,10,NULL);
}
