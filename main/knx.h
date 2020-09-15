#define PIN_KNX_RX                      17
#define PIN_KNX_TX                      16
#define KNX_HALF_BIT_TIME_MS            15

#define ESP_INTR_FLAG_DEFAULT 0

#define MAX_OCTETS 1000

unsigned char calc_even_parity_bit(unsigned char x);
unsigned char calc_odd_horizontal_parity_byte(int len, unsigned char* data);
#define SENDBUFFER_LEN 8
unsigned char sendbuffer[100];
static void isr_knx_tx_timer(void* arg);
static void isr_knx_rx_timer(void* arg);
void init_knx_tx_timer();
void init_knx_rx_timer();
static void IRAM_ATTR gpio_isr_handler(void* arg);
static void task_knxrx(void* arg);
static void task_knxtx(void* arg);
static void task_knx_send(void* arg);
void setup_knx_rx_pin();
void setup_knx_tx_pin();
void setup_knx_reading(     void (*knx_frame_received)(KNX_TP1_Frame frame)            );
void setup_knx_writing();