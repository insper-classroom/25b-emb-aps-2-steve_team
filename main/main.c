#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include <stdlib.h>

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 20;
const int I2C_SCL_GPIO = 21;
const uint BTN_PIN = 15; // Botão de mineração
const uint BTN_ANDAR = 14;
const uint BTN_PULAR = 13;

// Estrutura para a fila dos botões (btn_callback -> imu_task)
typedef struct {
    int pin;
    bool is_pressed;
} btn_event_t;

// Estrutura para a fila de comando (imu_task -> uart_task)
typedef struct {
    int axis; // 0=X, 1=Y/Z, 2=Click
    int val;
} command_t;

// typedef struct adc {
//     int axis;
//     int val;
// } adc_t;

// --- FILAS GLOBAIS ---
QueueHandle_t xQueueBtn;
QueueHandle_t xQueueComand;

// Variáveis para guardar o "vício" do giroscópio
int16_t gyro_error_x = 0;
int16_t gyro_error_z = 0;


void btn_callback(uint gpio, uint32_t events) {
    btn_event_t event;
    event.pin = gpio;
    
    // Verifica se foi borda de descida (apertou) ou subida (soltou)
    // Nota: Com pull-up, 0 = apertado, 1 = solto.
    if (gpio_get(gpio) == 0) {
        event.is_pressed = true;
    } else {
        event.is_pressed = false;
    }

    // Envia para a fila xQueueBtn. 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xQueueBtn, &event, &xHigherPriorityTaskWoken);
    
    // Força uma troca de contexto se necessário (para processar o botão rápido)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

}

void calibrate_mpu(void) {
    int16_t acceleration[3], gyro[3], temp;
    long sum_x = 0;
    long sum_z = 0;
    const int num_samples = 2000; // Lê 2000 vezes para ter precisão

    printf("Calibrando... Mantenha a MPU parada!\n");

    for (int i = 0; i < num_samples; i++) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        sum_x += gyro[0];
        sum_z += gyro[2];
        sleep_ms(1); // Pequeno delay entre leituras
    }

    // Calcula a média do erro
    gyro_error_x = sum_x / num_samples;
    gyro_error_z = sum_z / num_samples;

    printf("Calibracao concluida! Erro X: %d, Erro Z: %d\n", gyro_error_x, gyro_error_z);
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);



    // Configuração do Botão (Interrupção) MINERAR E ANDAR

    // MINERAR
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // ANDAR
    gpio_init(BTN_ANDAR);
    gpio_set_dir(BTN_ANDAR, GPIO_IN);
    gpio_pull_up(BTN_ANDAR);    

    // PULAR
    gpio_init(BTN_PULAR);
    gpio_set_dir(BTN_PULAR, GPIO_IN);
    gpio_pull_up(BTN_PULAR);

    // Habilita interrupção nas bordas de subida e descida
    gpio_set_irq_enabled_with_callback(BTN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled(BTN_ANDAR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BTN_PULAR, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    mpu6050_reset();
    calibrate_mpu();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3], temp;

    // adc_t envio_mpu;
    command_t cmd_packet;
    btn_event_t btn_data;

    while(1) {
        // --- PARTE 1: Verificar se há dados novos na fila dos botões ---
        // Usamos '0' no tempo de espera para NÃO bloquear a leitura da IMU.
        // Se tiver botão apertado, processa. Se não, segue pro giroscópio.
        if (xQueueReceive(xQueueBtn, &btn_data, 0) == pdTRUE) {
            // Aqui diferenciamos qual botão foi acionado
            if (btn_data.pin == BTN_PIN) {
                cmd_packet.axis = 2; // Canal 2 = Clique Mouse
            } 
            else if (btn_data.pin == BTN_ANDAR) {
                cmd_packet.axis = 3; // Canal 3 = Tecla W
            }
            else if (btn_data.pin == BTN_PULAR) { 
                cmd_packet.axis = 4; // Canal 4 = Pulo
            }

            // A lógica de apertado/solto é a mesma para os dois
            cmd_packet.val = btn_data.is_pressed ? 1 : 0;
            
            xQueueSend(xQueueComand, &cmd_packet, 0);
        }

        // leitura da MPU, sem fusao de dados
        mpu6050_read_raw(acceleration, gyro, &temp);

        gyro[0] -= gyro_error_x;
        gyro[2] -= gyro_error_z;

        FusionVector gyroscope = {
          .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
          .axis.y = gyro[1] / 131.0f,
          .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
          .axis.x = acceleration[0] / 16384.0f, // Conversão para g
          .axis.y = acceleration[1] / 16384.0f,
          .axis.z = acceleration[2] / 16384.0f,
        }; 

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // DeadZone
        if (abs(gyroscope.axis.x) < 2.0f) gyroscope.axis.x = 0;
        if (abs(gyroscope.axis.z) < 2.0f) gyroscope.axis.z = 0;

        // Envia eixo X
        cmd_packet.axis = 0;
        cmd_packet.val = -(int16_t)(gyroscope.axis.x * 10); 
        xQueueSend(xQueueComand, &cmd_packet, 0);

        // Envia eixo z
        cmd_packet.axis = 1;
        cmd_packet.val = (int16_t)(gyroscope.axis.z * 10);
        xQueueSend(xQueueComand, &cmd_packet, 0);
       

        // // --- LÓGICA DE CLIQUE (MAGNITUDE) ---
        // float accel_mag = sqrt(pow(accelerometer.axis.x, 2) + 
        //                        pow(accelerometer.axis.y, 2) + 
        //                        pow(accelerometer.axis.z, 2));


        // // Se a magnitude fugir de 1g (gravidade) em mais de 0.5g
        // if (fabs(accel_mag - 1.0f) > 0.5f) {
        //     envio_mpu.axis = 2;
        //     envio_mpu.val = 1;
        //     xQueueSend(xQueueComand, &envio_mpu, 0);
        //     // Delay para evitar múltiplos cliques
        //     vTaskDelay(pdMS_TO_TICKS(150));
        // }
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void uart_task(void *p){
    command_t received_cmd;

    while(1){
       
        if (xQueueReceive(xQueueComand, &received_cmd, portMAX_DELAY)){
            const uint8_t EOP = 0xFF;

            uint8_t axis_id = received_cmd.axis; 
            int16_t val = received_cmd.val;

            uint8_t LSB = (uint8_t)(val);
            uint8_t MSB = (uint8_t)(val >> 8);

            // Mandando o pacote 
            uart_putc_raw(uart0, (int)axis_id);
            uart_putc_raw(uart0, (int)LSB);
            uart_putc_raw(uart0, (int)MSB);
            uart_putc_raw(uart0, (int)EOP);
        }
    }
}

int main() {
    stdio_init_all();


    // Criação das Filas conforme diagrama
    xQueueBtn = xQueueCreate(10, sizeof(btn_event_t)); // Fila pequena, eventos são rápidos
    xQueueComand = xQueueCreate(32, sizeof(command_t));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_Task", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}
