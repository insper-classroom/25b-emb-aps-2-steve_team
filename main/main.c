#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include <math.h>

#include "Fusion.h"
#include <stdint.h>

#include "pins.h"

#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

QueueHandle_t xQueueBtn;
QueueHandle_t xQueueComand;

typedef enum {
    BTN1_LOOK = 1,   // olhar (MOVE contínuo enquanto segurado)
    BTN2_LMB,        // “bater” (detecção de swing)
    BTN3_RMB,        // “usar”
    BTN4_SHIFT,      // agachar
    BTN5_CTRL        // correr
} btn_id_t;

typedef struct {
    uint8_t btn;     // btn_id_t (1..5)
    uint8_t fall;    // 1=press (edge FALL), 0=release (edge RISE)
} BtnEvent;

typedef enum {
    CMD_MOVE = 1,    // payload: int16 dx, int16 dy (little-endian)
    CMD_LMB,         // flags bit0: 1=press, 0=release
    CMD_RMB,         // idem
    CMD_KEY_SHIFT,   // idem
    CMD_KEY_CTRL     // idem
} cmd_type_t;

typedef struct {
    uint8_t  type;   // cmd_type_t
    uint8_t  flags;  // bit0: 1=press, 0=release
    int16_t  dx;     // usado apenas quando type == CMD_MOVE
    int16_t  dy;     // usado apenas quando type == CMD_MOVE
} Comand;

// ===================== AJUSTES DE MAPA E HELPERS =====================
#define MOVE_GAIN_PITCH   2.0f     // ganho para pitch -> dx (ajuste depois)
#define MOVE_GAIN_ROLL    2.0f     // ganho para roll  -> dy (ajuste depois)
#define MOVE_DEADZONE_DEG 3.0f     // zona morta (graus) para evitar jitter
#define MOVE_MAX_ABS      1200     // saturação absoluta de dx/dy

// Swing (BTN2) - thresholds simples (ajuste depois):
#define SWING_ACCEL_Y_ABS 15000    // |accel[Y]| bruto (~g*16384 em ±2g defaults)
#define SWING_COOLDOWN_MS 120      // antirrebote de swing

static inline int16_t clamp_i16(int v, int lo, int hi) {
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

static inline int16_t f_to_i16(float x) {
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)(x);
}

// Conveniência para publicar comandos na fila xQueueComand
static inline void publish_move(int16_t dx, int16_t dy) {
    Comand c = { .type = CMD_MOVE, .flags = 0, .dx = dx, .dy = dy };
    xQueueSend(xQueueComand, &c, 0);
}
static inline void publish_key(uint8_t type, uint8_t press) {
    Comand c = { .type = type, .flags = (press ? 1u : 0u), .dx = 0, .dy = 0 };
    xQueueSend(xQueueComand, &c, 0);
}
// =====================================================================

// Converte o pino físico que gerou a IRQ no ID lógico do botão usado no firmware
static inline btn_id_t gpio_to_btn(uint gpio) {
    switch (gpio) {
        case GPIO_BTN1: return BTN1_LOOK;
        case GPIO_BTN2: return BTN2_LMB;
        case GPIO_BTN3: return BTN3_RMB;
        case GPIO_BTN4: return BTN4_SHIFT;
        case GPIO_BTN5: return BTN5_CTRL;
        default:        return 0; // GPIO não mapeado
    }
}

void btn_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    btn_id_t id = gpio_to_btn(gpio);
    if (id == 0) {
        return; // GPIO não mapeado
    }

    BtnEvent ev = { .btn = (uint8_t)id, .fall = 0 };

    // Convenção adotada: pull-up + ativo em nível baixo
    // FALL = PRESS  |  RISE = RELEASE
    if (events & GPIO_IRQ_EDGE_FALL) {
        ev.fall = 1;
        xQueueSendFromISR(xQueueBtn, &ev, &xHigherPriorityTaskWoken);
    }
    if (events & GPIO_IRQ_EDGE_RISE) {
        ev.fall = 0;
        xQueueSendFromISR(xQueueBtn, &ev, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void buttons_gpio_init(void) {
    const uint btn_pins[] = { GPIO_BTN1, GPIO_BTN2, GPIO_BTN3, GPIO_BTN4, GPIO_BTN5 };

    // Configura cada botão como entrada com pull-up (ativo em 0)
    for (size_t i = 0; i < sizeof(btn_pins)/sizeof(btn_pins[0]); i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }

    // Registra interrupções: um único callback global no RP2040
    // Primeiro pino: registra o callback
    gpio_set_irq_enabled_with_callback(
        btn_pins[0],
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        &btn_callback
    );

    // Demais pinos: apenas habilita IRQ (compartilham o mesmo callback)
    for (size_t i = 1; i < sizeof(btn_pins)/sizeof(btn_pins[0]); i++) {
        gpio_set_irq_enabled(btn_pins[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    }
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
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}



void mpu6050_task(void *p) {
    // I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    // (decisão atual: manter defaults do 6050 por enquanto)
    // mpu6050_configure(); // <- deixaremos para o polimento

    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    // ---- estados de botões
    bool btn_pressed[6] = {0}; // 1..5 usados
    bool swing_active = false; // já disparamos press do LMB?
    absolute_time_t swing_block_until = 0;

    while (1) {
        // 1) Consumir todos os eventos de botão acumulados na fila
        BtnEvent ev;
        while (xQueueReceive(xQueueBtn, &ev, 0) == pdTRUE) {
            uint8_t b = ev.btn;
            if (b >= 1 && b <= 5) {
                btn_pressed[b] = (ev.fall ? true : false); // FALL=press, RISE=release

                // Eventos imediatos para BTN3/4/5 (press/release)
                if (b == BTN3_RMB) {
                    publish_key(CMD_RMB, ev.fall);
                } else if (b == BTN4_SHIFT) {
                    publish_key(CMD_KEY_SHIFT, ev.fall);
                } else if (b == BTN5_CTRL) {
                    publish_key(CMD_KEY_CTRL, ev.fall);
                }

                // BTN2: se soltou → garante release do LMB e limpa swing
                if (b == BTN2_LMB && ev.fall == 0) {
                    if (swing_active) {
                        publish_key(CMD_LMB, 0); // release
                        swing_active = false;
                    }
                }
            }
        }

        // 2) Leitura IMU + atualização AHRS
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Conversões default (sem configurar ranges): gyro ±250 dps, accel ±2g
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        float pitch_deg = euler.angle.pitch; // +inclina pra frente
        float roll_deg  = euler.angle.roll;  // +inclina p/ direita

        // 3) Lógica do BTN2 (swing) tem prioridade sobre mirar (BTN1)
        bool btn2_hold = btn_pressed[BTN2_LMB];
        if (btn2_hold) {
            // durante hold, BTN1 (MOVE) fica suspenso
            // detectar swing simples: pico em |accel[Y]| com antirrebote
            if (!swing_active && get_absolute_time() > swing_block_until) {
                // fora do cooldown: pode detectar novo swing
                if (abs(acceleration[1]) > SWING_ACCEL_Y_ABS) {
                    publish_key(CMD_LMB, 1);      // press
                    swing_active = true;
                    // libera para um release quando soltar o botão
                    // inicia cooldown p/ não repetir durante mesmo golpe
                    swing_block_until = make_timeout_time_ms(SWING_COOLDOWN_MS);
                }
            }
        } else {
            // se BTN2 não está pressionado, garante estado consistente
            swing_active = false;
        }

        // 4) Lógica do BTN1 (olhar) — só quando BTN2 NÃO está ativo
        bool btn1_hold = btn_pressed[BTN1_LOOK];
        if (btn1_hold && !btn2_hold) {
            // aplicar deadzone e ganhos
            float dx_f = 0.0f, dy_f = 0.0f;
            if (fabsf(pitch_deg) > MOVE_DEADZONE_DEG) {
                dx_f = (pitch_deg > 0 ? pitch_deg - MOVE_DEADZONE_DEG : pitch_deg + MOVE_DEADZONE_DEG) * MOVE_GAIN_PITCH;
            }
            if (fabsf(roll_deg) > MOVE_DEADZONE_DEG) {
                dy_f = (roll_deg  > 0 ? roll_deg  - MOVE_DEADZONE_DEG : roll_deg  + MOVE_DEADZONE_DEG) * MOVE_GAIN_ROLL;
            }
            int16_t dx = clamp_i16(f_to_i16(dx_f), -MOVE_MAX_ABS, MOVE_MAX_ABS);
            int16_t dy = clamp_i16(f_to_i16(dy_f), -MOVE_MAX_ABS, MOVE_MAX_ABS);

            if (dx != 0 || dy != 0) {
                publish_move(dx, dy);
            }
        }
        // NOTA: se BTN1 solta ou BTN2 está ativo, simplesmente não publicamos MOVE.

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

static inline uint8_t checksum_sum8(uint8_t type, uint8_t flags, int16_t dx, int16_t dy) {
    uint16_t s = (uint16_t)type + (uint16_t)flags
               + (uint8_t)(dx & 0xFF) + (uint8_t)((dx >> 8) & 0xFF)
               + (uint8_t)(dy & 0xFF) + (uint8_t)((dy >> 8) & 0xFF);
    return (uint8_t)(s & 0xFF);
}

static void uart_send_cmd(const Comand *c) {
    uint8_t buf[8];
    buf[0] = 0xFF;
    buf[1] = c->type;
    buf[2] = c->flags;
    buf[3] = (uint8_t)(c->dx & 0xFF);
    buf[4] = (uint8_t)((c->dx >> 8) & 0xFF);
    buf[5] = (uint8_t)(c->dy & 0xFF);
    buf[6] = (uint8_t)((c->dy >> 8) & 0xFF);
    buf[7] = checksum_sum8(buf[1], buf[2], (int16_t)((buf[4]<<8)|buf[3]), (int16_t)((buf[6]<<8)|buf[5]));

    // Envio byte a byte (quando a UART estiver inicializada)
    uart_putc_raw(uart0, buf[0]);
    uart_putc_raw(uart0, buf[1]);
    uart_putc_raw(uart0, buf[2]);
    uart_putc_raw(uart0, buf[3]);
    uart_putc_raw(uart0, buf[4]);
    uart_putc_raw(uart0, buf[5]);
    uart_putc_raw(uart0, buf[6]);
    uart_putc_raw(uart0, buf[7]);
}

void uart_task(void *p){
    Comand cmd;
    while (1){
        if (xQueueReceive(xQueueComand, &cmd, portMAX_DELAY) == pdTRUE) {
            uart_send_cmd(&cmd);
        }
        // sem busy-loop: só acorda quando chegar algo
    }
}


int main() {
    stdio_init_all();

    xQueueBtn = xQueueCreate(8, sizeof(BtnEvent));
    xQueueComand = xQueueCreate(16, sizeof(Comand));

    buttons_gpio_init();   // <<< inicializa GPIOs e registra o callback

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 2048, NULL, 1, NULL);

    xTaskCreate(uart_task, "mpu4323446050_Task 1", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}