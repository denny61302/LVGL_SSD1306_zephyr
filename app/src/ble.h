#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)
#define INTERVAL_MIN  6     
#define INTERVAL_MAX  6  

int ble_init(void);
void ble_send(char *buffer, int len);