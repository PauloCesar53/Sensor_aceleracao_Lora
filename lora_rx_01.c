#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <string.h>
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "pico/binary_info.h"

//display 
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C
// Endereço padrão do MPU6050
static int addr = 0x68;

// Definição dos pinos I2C para o MPU6050
#define I2C_PORT_MPU i2c0                 // I2C0 usa pinos 0 e 1
#define I2C_SDA_MPU 0
#define I2C_SCL_MPU 1


ssd1306_t ssd;
// ============================================================================
// == Definições dos Pinos e SPI ==============================================
// ============================================================================
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_RST  20

// ============================================================================
// == Definições dos Registradores LoRa (RFM95/SX1276) ========================
// ============================================================================
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MAX_PAYLOAD_LENGTH   0x23
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

// ============================================================================
// == Modos de Operação (REG_OP_MODE) =========================================
// ============================================================================
#define MODE_SLEEP               0x80 // LoRa Mode + Sleep
#define MODE_STDBY               0x81 // LoRa Mode + Standby
#define MODE_TX                  0x83 // LoRa Mode + TX
#define MODE_RX_CONTINUOUS       0x85 // LoRa Mode + RX Continuous

// ============================================================================
// == Flags de Interrupção (REG_IRQ_FLAGS) ====================================
// ============================================================================
#define IRQ_TX_DONE_MASK         0x08

// Frequência de operação (915 MHz para o Brasil)
#define LORA_FREQUENCY_HZ 915000000
#define RF_CRYSTAL_FREQ_HZ 32000000

// Configurações de Potência
#define PA_BOOST_PIN             0x80 // Usar pino PA_BOOST para TX
#define OUTPUT_POWER_DBM         17   // Potência de saída em dBm (ex: 17 dBm)


// ============================================================================
// == Funções Básicas de Comunicação SPI ======================================
// ============================================================================

void rmf95_reset() {
    gpio_put(PIN_RST, 1);
    sleep_ms(1);
    gpio_put(PIN_RST, 0);
    sleep_ms(1);
    gpio_put(PIN_RST, 1);
    sleep_ms(5);
}

void rmf95_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx_data[] = { reg | 0x80, value };
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, tx_data, 2);
    gpio_put(PIN_CS, 1);
}

uint8_t rmf95_read_reg(uint8_t reg) {
    uint8_t tx_data[] = { reg & 0x7F, 0x00 };
    uint8_t rx_data[2];
    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx_data, rx_data, 2);
    gpio_put(PIN_CS, 1);
    return rx_data[1];
}

void rmf95_write_fifo(const uint8_t* buffer, uint8_t length) {
    uint8_t tx_data = REG_FIFO | 0x80;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &tx_data, 1); // Envia o endereço do FIFO com o bit de escrita
    spi_write_blocking(SPI_PORT, buffer, length); // Envia os dados do pacote
    gpio_put(PIN_CS, 1);
}

// ============================================================================
// == Funções de Configuração LoRa ============================================
// ============================================================================

void lora_set_frequency(long frequency) {
    uint64_t frf = ((uint64_t)frequency << 19) / RF_CRYSTAL_FREQ_HZ;
    rmf95_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    rmf95_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    rmf95_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_set_power(uint8_t power) {
    // Configura para usar o pino PA_BOOST, necessário para potências > 14 dBm
    rmf95_write_reg(REG_PA_CONFIG, PA_BOOST_PIN | (power - 2));
}

void lora_init() {
    // 1. Colocar em modo Sleep + LoRa
    rmf95_write_reg(REG_OP_MODE, MODE_SLEEP);
    sleep_ms(10);
    
    // 2. Configurar a frequência
    lora_set_frequency(LORA_FREQUENCY_HZ);

    // 3. Configurar a potência de saída
    lora_set_power(OUTPUT_POWER_DBM);

    // 4. Configurar os ponteiros da FIFO
    rmf95_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00); // TX usa a primeira metade da FIFO
    rmf95_write_reg(REG_FIFO_RX_BASE_ADDR, 0x80); // RX usa a segunda metade
    rmf95_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // 5. Configurar LNA (não crítico para TX, mas bom manter consistente)
    rmf95_write_reg(REG_LNA, 0x23);

    // 6. Configurar o modem (DEVE ser idêntico ao receptor)
    rmf95_write_reg(REG_MODEM_CONFIG_1, 0x72); // BW 125kHz, Coding Rate 4/5, Explicit Header
    rmf95_write_reg(REG_MODEM_CONFIG_2, 0x74); // SF7, CRC On
    rmf95_write_reg(REG_PREAMBLE_MSB, 0x00);
    rmf95_write_reg(REG_PREAMBLE_LSB, 0x08); // Preâmbulo de 8 símbolos

    // 7. Colocar em modo Standby para finalizar
    rmf95_write_reg(REG_OP_MODE, MODE_STDBY);
    sleep_ms(10);

    printf("RFM95 configurado para LoRa TX em %ld Hz\n", LORA_FREQUENCY_HZ);
}

// ============================================================================
// == Função de Transmissão LoRa ==============================================
// ============================================================================

void lora_send_packet(const char* message) {
    // 1. Mudar para o modo Standby para preparar
    rmf95_write_reg(REG_OP_MODE, MODE_STDBY);

    // 2. Apontar o FIFO para o início da área de TX e carregar os dados
    rmf95_write_reg(REG_FIFO_ADDR_PTR, 0x00); 
    rmf95_write_fifo((const uint8_t*)message, strlen(message));

    // 3. Definir o tamanho do payload que será enviado
    rmf95_write_reg(REG_PAYLOAD_LENGTH, strlen(message));
    
    // 4. Acionar a transmissão
    rmf95_write_reg(REG_OP_MODE, MODE_TX);
    
    // 5. Aguardar o fim da transmissão (polling na flag TxDone)
    while ((rmf95_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        sleep_ms(10);
    }
    
    // 6. Limpar a flag de interrupção
    rmf95_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    
    printf("Pacote enviado: '%s'\n", message);
}
// Função para resetar e inicializar o MPU6050
static void mpu6050_reset()
{
    // Dois bytes para reset: primeiro o registrador, segundo o dado
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT_MPU, addr, buf, 2, false);
    sleep_ms(100); // Aguarda reset e estabilização

    // Sai do modo sleep (registrador 0x6B, valor 0x00)
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT_MPU, addr, buf, 2, false);
    sleep_ms(10); // Aguarda estabilização após acordar
}

// Função para ler dados crus do acelerômetro, giroscópio e temperatura
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];

    // Lê aceleração a partir do registrador 0x3B (6 bytes)
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT_MPU, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT_MPU, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Lê giroscópio a partir do registrador 0x43 (6 bytes)
    val = 0x43;
    i2c_write_blocking(I2C_PORT_MPU, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT_MPU, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Lê temperatura a partir do registrador 0x41 (2 bytes)
    val = 0x41;
    i2c_write_blocking(I2C_PORT_MPU, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT_MPU, addr, buffer, 2, false);

    *temp = (buffer[0] << 8) | buffer[1];
}


// ============================================================================
// == Função Principal ========================================================
// ============================================================================

int main() {

    // Inicialização do display
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Inicialização da I2C do MPU6050
    i2c_init(I2C_PORT_MPU, 400 * 1000);
    gpio_set_function(I2C_SDA_MPU, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_MPU, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_MPU);
    gpio_pull_up(I2C_SCL_MPU);

    // Declara os pinos como I2C na Binary Info
    bi_decl(bi_2pins_with_func(I2C_SDA_MPU, I2C_SCL_MPU, GPIO_FUNC_I2C));
    mpu6050_reset();

    int16_t aceleracao[3], gyro[3], temp;

    stdio_init_all();
    sleep_ms(2000); 
    printf("Iniciando Transmissor LoRa (TX)...\n");

    // Inicialização do hardware
    spi_init(SPI_PORT, 1000 * 1000); // 1 MHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    
    // Reset e verificação do módulo
    rmf95_reset();
    uint8_t version = rmf95_read_reg(REG_VERSION);
    printf("Versao do RFM95: 0x%02X\n", version);

    if (version != 0x12) {
        printf("Falha na comunicacao SPI. Travando. ❌\n");
        //while(1);
    }
    printf("Comunicacao SPI OK! ✅\n");

    // Configura o rádio para operar com LoRa
    lora_init();
    
    int counter = 0;
    char message_buffer[50];

    while (1) {
        // Leitura dos dados de aceleração, giroscópio e temperatura
        mpu6050_read_raw(aceleracao, gyro, &temp);

        // Conversão para unidade de 'g'
        float ax = aceleracao[0] / 16384.0f;
        float ay = aceleracao[1] / 16384.0f;
         // Montagem das strings para o display
        char acelx[20];
        char acely[20];

        
        snprintf(acelx,  sizeof(acelx),  "%5.1f", ax);
        snprintf(acely, sizeof(acely), "%5.1f", ay);
        ssd1306_fill(&ssd, 0); // limpa o display
        ssd1306_draw_string(&ssd, "Acel. X", 18, 4);
        ssd1306_draw_string(&ssd, acelx, 18, 13);         // Exibe acel x
        ssd1306_draw_string(&ssd, "Acel. Y", 18, 41);
        ssd1306_draw_string(&ssd, acely, 18, 50);         // Exibe acel y
        ssd1306_rect(&ssd, 3, 3, 122, 60, 1, 0); // Desenha um retângulo
        //ssd1306_line(&ssd, 3, 22, 123, 22, 1);   // Desenha uma linha
        ssd1306_send_data(&ssd);
        // Monta a mensagem a ser enviada
        //snprintf(message_buffer, sizeof(message_buffer), "Ola do Pico! Cont: %d", counter);
        snprintf(message_buffer, sizeof(message_buffer), "AcelX: %.2f, AcelY: %.2f", ax, ay);
        // Envia o pacote
        //lora_send_packet(message_buffer);
        //counter++;
        sleep_ms(200);
        //sleep_ms(5000); // Espera 5 segundos antes de enviar o próximo pacote
    }

    return 0;
}