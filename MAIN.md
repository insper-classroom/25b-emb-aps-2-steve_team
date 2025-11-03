# Documentação Completa do Projeto: Controle com MPU6050 e FreeRTOS

## 🧭 Visão Geral

Este projeto implementa um sistema embarcado utilizando o **Raspberry Pi Pico** e o sensor **MPU6050** para controle de movimentos e ações, simulando o uso de um *martelo controlador* que interage com o computador via UART. A comunicação é feita por um protocolo binário simples e o processamento dos dados de movimento utiliza a biblioteca **Fusion AHRS**.

O sistema foi desenvolvido com **FreeRTOS**, utilizando tarefas (*tasks*), filas (*queues*) e interrupções (*ISRs*) para garantir execução concorrente e responsiva.

---

## ⚙️ Estrutura do Código

### 1. Inclusão de Bibliotecas

Inclui bibliotecas essenciais para:

* **FreeRTOS:** gerenciamento multitarefa e comunicação entre tarefas.
* **Pico SDK:** controle de GPIO, I2C e UART.
* **Fusion AHRS e MPU6050:** processamento dos dados inerciais.
* **`pins.h`:** definição dos pinos utilizados no hardware.

---

### 2. Definições Globais

#### a) Configurações gerais

```c
#define SAMPLE_PERIOD (0.01f) // período de amostragem de 100 Hz
```

#### b) Endereços e GPIOs do I2C

```c
const int MPU_ADDRESS = 0x68; // endereço padrão do MPU6050
const int I2C_SDA_GPIO = 4;   // pino SDA
const int I2C_SCL_GPIO = 5;   // pino SCL
```

---

### 3. Estruturas e Enumerações

#### a) Enumeração dos botões (`btn_id_t`)

```c
typedef enum {
    BTN1_LOOK = 1,   // olhar (mover câmera)
    BTN2_LMB,        // clique esquerdo (bater)
    BTN3_RMB,        // clique direito (usar item)
    BTN4_SHIFT,      // agachar
    BTN5_CTRL        // correr
} btn_id_t;
```

#### b) Estrutura de eventos de botão (`BtnEvent`)

```c
typedef struct {
    uint8_t btn;  // ID do botão (1..5)
    uint8_t fall; // 1 = pressionado, 0 = solto
} BtnEvent;
```

#### c) Estrutura de comandos enviados pela UART (`Comand`)

```c
typedef struct {
    uint8_t type;   // tipo de comando (MOVE, LMB, etc.)
    uint8_t flags;  // 1 = press, 0 = release
    int16_t dx;     // deslocamento X (para MOVE)
    int16_t dy;     // deslocamento Y (para MOVE)
} Comand;
```

---

### 4. Filas e Comunicação entre Tarefas

```c
QueueHandle_t xQueueBtn;    // Fila de eventos dos botões
QueueHandle_t xQueueComand; // Fila de comandos para UART
```

* **xQueueBtn:** recebe eventos gerados pela ISR dos botões.
* **xQueueComand:** recebe comandos gerados pela task da IMU para envio pela UART.

---

### 5. Interrupções dos Botões (GPIO)

#### a) Callback Global (`btn_callback`)

```c
void btn_callback(uint gpio, uint32_t events) {
    BtnEvent ev = {...};
    if (events & GPIO_IRQ_EDGE_FALL) ev.fall = 1; // Pressionado
    if (events & GPIO_IRQ_EDGE_RISE) ev.fall = 0; // Solto
    xQueueSendFromISR(xQueueBtn, &ev, ...);
}
```

Cada vez que um botão é pressionado ou solto, um evento é enviado para a fila de botões.

#### b) Configuração dos Pinos (`buttons_gpio_init`)

```c
static void buttons_gpio_init(void) {
    gpio_init(...);
    gpio_set_dir(...);
    gpio_pull_up(...);
    gpio_set_irq_enabled_with_callback(...);
}
```

Define todos os botões como entradas com *pull-up* e ativa as interrupções para detecção de bordas de subida e descida.

---

### 6. Leitura e Processamento do MPU6050

#### a) Reset do Sensor

```c
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}
```

Ativa o MPU6050 e o retira do modo *sleep*.

#### b) Leitura de Dados Brutos

```c
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
```

Lê os valores de aceleração e giroscópio e os armazena nos vetores correspondentes.

#### c) Processamento AHRS (Fusion)

```c
FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
```

Calcula a orientação (pitch e roll) a partir dos dados do MPU6050.

---

### 7. Task Principal: `mpu6050_task`

Responsável por:

* Ler o estado dos botões.
* Processar dados da IMU.
* Enviar comandos para a UART.

#### Lógica dos Botões

| Botão | Função | Ação                            |
| ----- | ------ | ------------------------------- |
| BTN1  | Olhar  | Move a câmera (MOVE)            |
| BTN2  | Bater  | Detecta swing (clique esquerdo) |
| BTN3  | Usar   | Clique direito                  |
| BTN4  | Shift  | Agachar                         |
| BTN5  | Ctrl   | Correr                          |

#### Priorização

Quando BTN2 (bater) está pressionado, BTN1 (olhar) é temporariamente desativado para evitar conflito entre movimentos.

---

### 8. Comunicação UART

Cada comando (`Comand`) é convertido em um *frame binário de 8 bytes*:

```
[0] 0xFF  - byte de sincronização
[1] type  - tipo de comando
[2] flags - estado (press/release)
[3] dx_lo - byte baixo do deslocamento X
[4] dx_hi - byte alto do deslocamento X
[5] dy_lo - byte baixo do deslocamento Y
[6] dy_hi - byte alto do deslocamento Y
[7] checksum - soma módulo 256 dos bytes 1–6
```

#### Função de Envio

```c
static void uart_send_cmd(const Comand *c) {
    uint8_t buf[8];
    uart_putc_raw(uart0, buf[i]); // envia byte a byte
}
```

---

### 9. Task UART

```c
void uart_task(void *p) {
    Comand cmd;
    while (1) {
        if (xQueueReceive(xQueueComand, &cmd, portMAX_DELAY) == pdTRUE) {
            uart_send_cmd(&cmd);
        }
    }
}
```

Envia continuamente os comandos recebidos pela fila `xQueueComand`.

---

### 10. Função `main()`

Responsável pela inicialização geral:

```c
int main() {
    stdio_init_all();

    xQueueBtn = xQueueCreate(8, sizeof(BtnEvent));
    xQueueComand = xQueueCreate(16, sizeof(Comand));

    buttons_gpio_init();

    xTaskCreate(mpu6050_task, ...);
    xTaskCreate(uart_task, ...);

    vTaskStartScheduler();
}
```

---

## 🔄 Fluxo Completo do Sistema

```text
[Botão físico]
   ↓ (Interrupção GPIO)
[btn_callback] → xQueueBtn
   ↓ (Task MPU6050)
[Lê IMU + interpreta botões]
   ↓
[Gera Comand → xQueueComand]
   ↓
[uart_task]
   ↓
[Envia via UART → PC (Python)]
```

---

## 🧩 Principais Componentes e Responsabilidades

| Componente         | Responsabilidade                            |
| ------------------ | ------------------------------------------- |
| **ISR GPIO**       | Detecta press/release e envia `BtnEvent`.   |
| **Task MPU6050**   | Lê sensores e interpreta botões.            |
| **Task UART**      | Empacota e envia comandos.                  |
| **xQueueBtn**      | Comunicação ISR → Task MPU6050.             |
| **xQueueComand**   | Comunicação Task MPU6050 → Task UART.       |
| **Fusion AHRS**    | Converte aceleração/giroscópio em ângulos.  |
| **Protocolo UART** | Comunicação binária eficiente com checksum. |

---

## ✅ Conclusão

O código implementa uma arquitetura modular e robusta com:

* Separação clara entre leitura, processamento e comunicação;
* Sincronização eficiente via filas do FreeRTOS;
* Protocolo UART binário seguro e extensível;
* Flexibilidade para ajustes de sensibilidade e jogabilidade.

O sistema está totalmente compatível com a aplicação Python que interpreta os frames e converte as ações em comandos reais de mouse e teclado.
