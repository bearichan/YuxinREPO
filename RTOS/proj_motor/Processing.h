#define IR01ID 4
#define IR23ID 4
#define PINGID 4
#define RCV01  2
#define RCV23  2
#define RCVPING 2

typedef struct SensorMSG {
	unsigned long id;
	unsigned long data1;
	unsigned long data2;
} SensorMSG;

void Processing_Init(void);
