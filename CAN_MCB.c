#include "include/CAN_MCB.h"
#include "string.h"
#include "../../../include/main.h"              // TODO: mirar este def
#include "esp_log.h"

extern QueueSetHandle_t queueMotorControl; 
config_init_mcb_t mcbConfigInit;
rx_motor_control_board_t motorControlBoard;

static QueueHandle_t spp_uart_queue;

const char *TAG = "CAN_MCB";

#define PATTERN_CHR_NUM 1

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

static void processMCBData(uint8_t *data) {
    rx_motor_control_board_t rxData;
    uint16_t newChecksum = 0,lentghRx = 0;

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


    SerialFeedback MCBPacket;
    memcpy(&MCBPacket,data,sizeof(SerialFeedback));
    // printf("Data leida: start: %x, cmd1: %x, cmd2: %x, spdL: %x, spdR: %x, %x, %x, %x, %x\n",MCBPacket.start,MCBPacket.cmd1,MCBPacket.cmd2,MCBPacket.speedR_meas,MCBPacket.speedL_meas,MCBPacket.batVoltage,MCBPacket.boardTemp,MCBPacket.cmdLed,MCBPacket.checksum);
    // printf("start struct: %02X\n",MCBPacket.start);

    if( MCBPacket.start == START_CODE_HEADER) {

        newChecksum = (uint16_t)(MCBPacket.start ^ MCBPacket.cmd1 ^ MCBPacket.cmd2 ^ MCBPacket.speedR_meas ^ MCBPacket.speedL_meas 
                                       ^ MCBPacket.batVoltage ^ MCBPacket.boardTemp ^ MCBPacket.cmdLed);
        if( newChecksum == MCBPacket.checksum){
            // memcpy(&motorControlBoard,&MCBPacket,sizeof(SerialFeedback));
            // printf("Data nueva copiada: %d",motorControlBoard.status);

            printf("Nuevo paquete OK -> voltage: %d\tspeedL: %d\tspeedR:%d\n",MCBPacket.batVoltage,MCBPacket.speedL_meas,MCBPacket.speedR_meas);
        }
        else {
            printf("error checksum\n");
        }
    }
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
            
            switch(event.type) {
            //  case UART_DATA:
            //     if (event.size) {
            //         // printf("Data recibida: %d\n",event.size);
            //         uint8_t length = uart_read_bytes(UART_NUM_2, data, event.size, 0);
            //         printf("Data recibida: %d\n",length);
            //         if (length >= sizeof(SerialFeedback) && data[0] == 0xcd) {
            //             // Procesar los datos recibidos
            //             printf("\nNuevo paquete:\n");
            //             ESP_LOG_BUFFER_HEX(TAG, data, length);
            //             processMCBData(data, length);
            //         }
            //         else {
            //             uart_flush_input(UART_NUM_2);       // out of sync, limpio uart
            //             printf("\nout of sync\n");
            //         }
            //     }
            //     break;
                 //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(spp_uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(spp_uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:

                size_t sizeBuffer;
                uart_get_buffered_data_len(UART_NUM_2, &sizeBuffer);
                if (sizeBuffer < sizeof(SerialFeedback)) { 
                    break;                                                  // wait for the correct size
                }
                int pos = uart_pattern_pop_pos(UART_NUM_2);                 // get the position of the pattern
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(UART_NUM_2);
                } else {
                    uint8_t discard[50];         // TODO: mejorar metodo de descarte, estoy usando memoria inutilmente
                    uart_read_bytes(UART_NUM_2, discard, pos,0);   // just to discard previous bytes
                    uart_read_bytes(UART_NUM_2, data, sizeof(SerialFeedback),0);
                    // printf("Data leida desde el pattern: ");
                    // ESP_LOG_BUFFER_HEX(TAG, data, sizeof(data));

                    // printf("Data limpia procesada: ");
                    // ESP_LOG_BUFFER_HEX(TAG, data, sizeof(SerialFeedback));

                    processMCBData(data);
                    uart_flush_input(UART_NUM_2);
                }
                break;
                default:
                    ESP_LOGI(TAG, "unhandled event type: %d", event.type);
                break;
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

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(UART_NUM_2, 0xCD, PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_NUM_2, 20);

    printf("\nMCB INIT\n");

    xTaskCreatePinnedToCore(controlHandler,"MCB handler task",4096,NULL,10,NULL,config->core);
}
