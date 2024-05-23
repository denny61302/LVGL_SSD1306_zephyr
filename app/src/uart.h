#define UART_BUF_SIZE 40
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX 50000
#define CMD_MSG_SIZE 82

int uart_init(void);
void uart_send(char *buffer);