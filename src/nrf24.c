//
//	Receive and process lofi security packets
//	on ESP8266
//

#include "esp_common.h"
#include "gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#include "uart.h"
#include "dmsg.h"
#include "xmit.h"

#include "MQTTESP8266.h"
#include "MQTTClient.h"


#if 0
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <PubSubClient.h>
//#include <Wire.h>
#include <SPI.h>
#include <stdio.h>
#include <stdarg.h>


#define DEBUG   1
#if DEBUG
#define dbug_printf(fmt, ...) printf(fmt, __VA_ARGS__)
#define dbug_puts(str) puts(str)
#define dbug_putchar(c) putchar(c)
#else
#define dbug_printf(fmt, ...) do {} while (0)
#define dbug_puts(str)
#define dbug_putchar(c)
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#define NRFIRQ			  16
#define nrfCSN        5
#define nrfCE         17

#define MOSI_PIN      23
#define MISO_PIN      19
#define SCLK_PIN      18
#endif

#define ETS_GPIO_INTR_ENABLE() _xt_isr_unmask(1 << ETS_GPIO_INUM)
#define ETS_GPIO_INTR_DISABLE() _xt_isr_mask(1 << ETS_GPIO_INUM)

#define nrfIRQ_IO_MUX	PERIPHS_IO_MUX_MTCK_U
#define nrfIRQ_IO_NUM	5
#define nrfIRQ_IO_FUNC	FUNC_GPIO5
#define nrfIRQ_IO_PIN	GPIO_Pin_5
#define nrfIRQ        5
#define nrfCSN        15
#define nrfCE         4

#define nrfMOSI      13
#define nrfMISO      12
#define nrfSCK       14

#define ASSERT_CSN()	GPIO_OUTPUT_SET(nrfCSN, 0)
#define DEASSERT_CSN()	GPIO_OUTPUT_SET(nrfCSN, 1)
#define ASSERT_CE()		GPIO_OUTPUT_SET(nrfCE, 1)
#define DEASSERT_CE()	GPIO_OUTPUT_SET(nrfCE, 0)
#define ASSERT_MOSI()	GPIO_OUTPUT_SET(nrfMOSI, 1)
#define DEASSERT_MOSI()	GPIO_OUTPUT_SET(nrfMOSI, 0)
#define ASSERT_SCK()	GPIO_OUTPUT_SET(nrfSCK, 1)
#define DEASSERT_SCK()	GPIO_OUTPUT_SET(nrfSCK, 0)


#define PAYLOAD_LEN		3


#define NRF_CONFIG			  0x00
#define NRF_EN_AA			    0x01
#define NRF_EN_RXADDR		  0x02
#define NRF_SETUP_AW		  0x03
#define NRF_SETUP_RETR		0x04
#define NRF_RF_CH			    0x05
#define NRF_RF_SETUP		  0x06
#define NRF_STATUS			  0x07
#define NRF_OBSERVE_TX		0x08
#define NRF_CD				    0x09
#define NRF_RX_ADDR_P0		0x0A
#define NRF_RX_ADDR_P1		0x0B
#define NRF_RX_ADDR_P2		0x0C
#define NRF_RX_ADDR_P3		0x0D
#define NRF_RX_ADDR_P4		0x0E
#define NRF_RX_ADDR_P5		0x0F
#define NRF_TX_ADDR			  0x10
#define NRF_RX_PW_P0		  0x11
#define NRF_RX_PW_P1		  0x12
#define NRF_RX_PW_P2		  0x13
#define NRF_RX_PW_P3		  0x14
#define NRF_RX_PW_P4		  0x15
#define NRF_RX_PW_P5		  0x16
#define NRF_FIFO_STATUS		0x17
#define NRF_DYNPD			    0x1C
#define NRF_FEATURE			  0x1D

#if (!defined(TRUE))
#define TRUE 1
#endif
	
#if (!defined(FALSE))
#define FALSE 1
#endif


typedef enum {
	SENID_NONE = 0,
	SENID_SW1,
	SENID_SW2,
	SENID_VCC,
	SENID_TEMP,
	SENID_CTR,
	SENID_REV
} senId_t;


typedef enum {
	speed_1M = 0,
	speed_2M = 1,
	speed_250K = 2
} speed_t;


xSemaphoreHandle sema_nrf;
extern xQueueHandle publish_queue;

//long lastMsg = 0;
//char msg[50];
//int value = 0;

bool verbose = true;
bool printMqtt = true;
bool longStr = false;
bool printPayload = false;
bool printSeq = false;
bool en_shockburst = true;
speed_t speed = speed_250K;
int rf_chan = 84;
//int maxNodeRcvd = 0;
uint8_t nrfConfigVal = 0x3C; 
volatile uint8_t   gNrfStatus;
//volatile int gError = 0;

char *nodeMap[] = {
	"node/0",
	"node/1",
	"node/2",
	"node/3",
	"node/4",
	"door/GarageN",			// node/5
	"node/6",
	"node/7",
	"door/Hall",		// node/8
	"node/9",
	"node/10",
	"node/11",
	"node/12",
	"node/13",
	"door/Garage",		// node/14
	"node/15",
	"node/16",
	"door/GarageS",		// node/17
	"door/Sliding",		// node/18
	"door/Back",		// node/19
	"node/20",
	"window/officeN",	// node/21
	"node/22",		// node/22
	"node/23",
	"window/officeS",	// node/24
	"door/GarageS",	// node/25
	"window/masterE",	// node/26
	"node/27",		// node/27
	"door/Front",
	"node/29",
	"node/30",
	"node/31",
	"door/Sliding",
	"node/33",
	"node/34",
	"node/35",
	"node/36",
	"node/37",
	"node/38",
	"node/99"
};
int maxNodes = sizeof(nodeMap)/sizeof(char*);


//************  Forward Declarations
void parsePayload( void );
uint8_t spiXfer( uint8_t *buf, int cnt );
uint8_t nrfRegRead( int reg );
uint8_t nrfRegWrite( int reg, int val );
void nrfPrintDetails(void);
int nrfAvailable( uint8_t *pipe_num );
int nrfReadPayload( uint8_t *payload, int len );
uint8_t nrfFlushTx( void );
uint8_t nrfFlushRx( void );
int nrfAddrRead( uint8_t reg, uint8_t *buf, int len );
uint8_t nrfReadRxPayloadLen(void);
void IRAM_ATTR nrfIntrHandler(void);



//
// nrfIRQ Interrupt Handler
//
void IRAM_ATTR nrfIntrHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint32 status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);

	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, status);

	if (status & nrfIRQ_IO_PIN) {
		nrfRegWrite( NRF_STATUS, 0x70 ); // clear the interrupt
		xSemaphoreGiveFromISR( sema_nrf, &xHigherPriorityTaskWoken );
	}
}

//
// Configure the NRF24l01p peripheral
//
void nrfConfig(void)
{
	// set CSN, CE, SCK, and MOSI as GPIO outputs
	PIN_FUNC_SELECT(GPIO_PIN_REG_15, FUNC_GPIO15);
	DEASSERT_CSN();
	PIN_FUNC_SELECT(GPIO_PIN_REG_4, FUNC_GPIO4);
	DEASSERT_CE();
	PIN_FUNC_SELECT(GPIO_PIN_REG_13, FUNC_GPIO13);
	DEASSERT_MOSI();
	PIN_FUNC_SELECT(GPIO_PIN_REG_14, FUNC_GPIO14);
	DEASSERT_SCK();

	// set MISO as GPIO input
	PIN_FUNC_SELECT(GPIO_PIN_REG_12, FUNC_GPIO12);
	GPIO_AS_INPUT(GPIO_PIN_REG_12);


	// NRF setup
	// enable 16-bit CRC; mask TX_DS and MAX_RT
	nrfRegWrite( NRF_CONFIG, nrfConfigVal); //0x3c );

	if (en_shockburst) {
		// set nbr of retries and delay
		nrfRegWrite( NRF_SETUP_RETR, 0x33 );
		nrfRegWrite( NRF_EN_AA, 1 ); // enable auto ack
	} else {
		nrfRegWrite( NRF_SETUP_RETR, 0 );
		nrfRegWrite( NRF_EN_AA, 0 );
	}

	// Disable dynamic payload
	nrfRegWrite( NRF_FEATURE, 0);
	nrfRegWrite( NRF_DYNPD, 0);

	// Reset STATUS
	nrfRegWrite( NRF_STATUS, 0x70 );

	nrfRegWrite( NRF_EN_RXADDR, 1 ); //3);
	nrfRegWrite( NRF_RX_PW_P0, PAYLOAD_LEN );

	nrfRegWrite( NRF_RX_PW_P1, 0 ); //PAYLOAD_LEN );
	nrfRegWrite( NRF_RX_PW_P2, 0 );
	nrfRegWrite( NRF_RX_PW_P3, 0 );
	nrfRegWrite( NRF_RX_PW_P4, 0 );
	nrfRegWrite( NRF_RX_PW_P5, 0 );

	// Set up channel
	nrfRegWrite( NRF_RF_CH, rf_chan );

	int speedVal = 0x0e;
	switch (speed) {
	case speed_1M:
    speedVal = 0x06;
		break;
	case speed_250K:
    speedVal = 0x26;
		break;
	}
	nrfRegWrite( NRF_RF_SETUP, speedVal | 0);

	nrfFlushTx();
	nrfFlushRx();

	//
	// Configure nrfIRQ as GPIO input and negative edge triggered
	// And configure the Interrupt Handler
	//
	GPIO_ConfigTypeDef	io_in_conf;
	io_in_conf.GPIO_IntrType = GPIO_PIN_INTR_NEGEDGE;
	io_in_conf.GPIO_Mode = GPIO_Mode_Input;
	io_in_conf.GPIO_Pin = nrfIRQ_IO_PIN;
	io_in_conf.GPIO_Pullup = GPIO_PullUp_EN;
	gpio_config(&io_in_conf);

	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, TRUE);
	gpio_intr_handler_register(nrfIntrHandler, NULL);
//	gpio16_output_set(1);
	ETS_GPIO_INTR_ENABLE();

  
	// Power up radio and delay 5ms
	nrfRegWrite( NRF_CONFIG, nrfConfigVal | 0x02); //nrfRegRead( NRF_CONFIG ) | 0x02 );
	vTaskDelay(5);

	// Enable PRIME RX (PRX)
	nrfRegWrite( NRF_CONFIG, nrfConfigVal | 0x03); //nrfRegRead( NRF_CONFIG ) | 0x01 );

	if (verbose)
		nrfPrintDetails();

	ASSERT_CE();

}

void ICACHE_FLASH_ATTR
nrf_task(void * pvParameters)
{

	vTaskDelay(2000/portTICK_RATE_MS);
	vSemaphoreCreateBinary(sema_nrf);
	nrfConfig();

	while (1) {
		xSemaphoreTake(sema_nrf, portMAX_DELAY);
		parsePayload();
	}
}


#define TBUF_LEN        80
#define TOPIC_LEN       80
#define TOPIC_VAL_LEN   20

void parsePayload( void )
{
  int i;
	unsigned short val;
	uint8_t	sensorId;
	uint8_t nodeId;
	char tbuf[TBUF_LEN+1];
	char topic[TOPIC_LEN];
	char topicVal[TOPIC_VAL_LEN];
//	int topicIdx;
	int	tbufIdx;
	int	seq;
  unsigned char payload[PAYLOAD_LEN];
  int pkt_avail = false;

  
//  for (;;) {
    
    // See if there's a message in the queue (block forever)
//    xQueueReceive(pldQ, (void *)payload, portMAX_DELAY);


 
//    if (!pkt_avail)
//      xSemaphoreTake( sema_nrf, portMAX_DELAY );

//    payLen = nrfReadRxPayloadLen();
//    if (payLen != PAYLOAD_LEN) {
//      printf("PAYLOAD LEN: %d\n", payLen);
//      continue;
//    }

    nrfReadPayload( payload, 3 );

    if (printPayload) {
      dmsg_printf("Payload: %02X %02X %02X\n", payload[0], payload[1], payload[2]);
      return;
    }

//	  tbuf[0] = '\0';
//    tbufIdx = 0;
//	  topic[0] = '\0';
//    topicIdx = 0;
//    topicVal[0] = '\0';

    if (payload[1] == 0)
      return;

    nodeId = payload[0];

	  if (nodeId < 1 || nodeId >= maxNodes) {
		  dmsg_printf("Bad nodeId: %d\n", nodeId);
		  return;
	  }

    sensorId = (payload[1]>>4) & 0xF;
    seq = (payload[1] >> 2) & 0x3;

    strcpy(topic, "lofi/");
    strcat(topic, nodeMap[nodeId]);
    
	  if (longStr) {
			tbufIdx = snprintf(&tbuf[0], TBUF_LEN, "Id: %2d", nodeId);
	  }

    if (longStr && printSeq) {
      tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Seq: %1d", seq);
    }

    val = payload[1] & 0x03;
    val <<= 8;
    val += payload[2];

		switch (sensorId) {
		case SENID_REV:
      strcat(topic, "/rev");
      sprintf(topicVal, "%d", val);
			if (longStr) {
			 tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Rev: %4d", val);
			}
			break;
		case SENID_CTR:
      strcat(topic, "/ctr");
      sprintf(topicVal, "%d", val);
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Ctr: %4d", val);
			}
			break;
		case SENID_SW1:
      strcat(topic, "/sw1");
      sprintf(topicVal, (payload[1] & 0x02) ? "OPEN" : "SHUT");
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW1: %s", (payload[1] & 0x02) ? "OPEN" : "SHUT");
			}
			//i++;
			break;
		case SENID_SW2:
      strcat(topic, "/sw1");
      sprintf(topicVal, (payload[1] & 0x02) ? "OPEN" : "SHUT");
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW2: %s", (payload[1] & 0x02) ? "OPEN" : "SHUT");
			}
			//i++;
			break;
		case SENID_VCC:
      strcat(topic, "/vcc");
      sprintf(topicVal, "%4.2f", (1.1 * 1024.0)/(float)val);
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Vcc: %4.2f",(1.1 * 1024.0)/(float)val);
			}
			break;
		case SENID_TEMP:
      strcat(topic, "/temp");
      sprintf(topicVal, "%4.2f", 1.0 * (float)val - 260.0);
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Temp: %4.2f",1.0 * (float)val - 260.0);
			}
			break;
		default:
			dmsg_printf("Bad SensorId: %d\n", sensorId);
			return;
			break;
		}

    if (longStr) {
      dmsg_printf("%s", tbuf);
    } else {
      dmsg_printf("Id: %2d ", nodeId);
      dmsg_printf("Seq: %d ", seq);
      dmsg_printf("%s %s", topic, topicVal);
    }
    
    if (sensorId == SENID_SW1)
      dmsg_printf(" %s\n", (payload[1] & 0x01) ? "PC" : "");
    else
      dmsg_puts("");

		strcat(topic, ",");
		strcat(topic, topicVal);
		if (xQueueSend(publish_queue, (void *)topic, 0) == pdFALSE) {
			dmsg_puts("Publish queue overflow.");
		}

    pkt_avail = ((nrfRegRead(NRF_FIFO_STATUS) & 1) == 0);
//    printf( "parsePayload MEMORY WATERMARK %d\n", uxTaskGetStackHighWaterMark(NULL) );
//  }
}


uint8_t spi_transfer(uint8_t tx)
{
	register uint8_t i;
	register uint8_t rx = tx;

	for (i = 0x80; i; i >>= 1) {
		if (tx & i) {
			ASSERT_MOSI();
		} else {
			DEASSERT_MOSI();
		}

		ASSERT_SCK();

		if (gpio_input_get() & 0x1000) {
			rx |= i;
		}

		DEASSERT_SCK();
	}
	return rx;
}

uint8_t spiXfer(uint8_t *buf, int cnt)
{
	ASSERT_CSN();

	for (int i = 0; i < cnt; i++) {
		buf[i] = spi_transfer(buf[i]);
	}

	DEASSERT_CSN();

//	gNrfStatus = buf[0];
	return buf[0];
}


int nrfAddrRead( uint8_t reg, uint8_t *buf, int len )
{
	if (buf && len > 1) {
		buf[0] = reg & 0x1f;
		spiXfer(buf, len+1);
		return buf[1];
	}
	return -1;
}


uint8_t nrfFlushRx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xE2;
	return spiXfer(spiBuf, 1);
}

uint8_t nrfFlushTx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xE1;
	return spiXfer(spiBuf, 1);
}

uint8_t nrfRegWrite( int reg, int val)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x20 | (reg & 0x1f);
	spiBuf[1] = val;
	return spiXfer(spiBuf, 2);
}

uint8_t nrfRegRead( int reg )
{
	uint8_t spiBuf[2];

	spiBuf[0] = reg & 0x1f;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	return spiBuf[1];
}

uint8_t nrfReadRxPayloadLen(void)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x60;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	return spiBuf[1];
}

int nrfAvailable( uint8_t *pipe_num )
{
	uint8_t status;

	status = nrfRegRead( NRF_STATUS );
	if (status & 0x40 ) {
		if ( pipe_num ) {
			*pipe_num = ((status>>1) & 0x7);
		}
		return 1;
	}
	return 0;
}

int nrfReadPayload( uint8_t *payload, int len )
{
	uint8_t spiBuf[33];
	int i;

	if (len > 32)
		return -1;
	if (len < 1)
		return -1;

	spiBuf[0] = 0x61;
	for (i = 1; i < len+1; i++)
		spiBuf[i] = 0;
	spiXfer(spiBuf, len+1);
	if (payload)
		for (i = 1; i < len+1; i++)
			payload[i-1] = spiBuf[i];
	
	return 0;
}

void nrfPrintDetails(void)
{
	uint8_t		buf[6];

	dmsg_puts("================ SPI Configuration ================" );
	dmsg_printf("CSN Pin  \t = Custom GPIO%d\n", nrfCSN  );
	dmsg_printf("CE Pin  \t = Custom GPIO%d\n", nrfCE );
	dmsg_puts("Clock Speed\t = " );
	dmsg_puts("1 Mhz");
	dmsg_puts("================ NRF Configuration ================");
 

	dmsg_printf("STATUS: %02X\n", nrfRegRead( NRF_STATUS ));
	bzero(buf, 6);
	nrfAddrRead( NRF_RX_ADDR_P0, buf, 5 );
	dmsg_printf("RX_ADDR_P0: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//	printf("RX_ADDR_P0: %02X\n", nrfRegRead( NRF_RX_ADDR_P0 ));
	bzero(buf, 6);
	nrfAddrRead( NRF_RX_ADDR_P1, buf, 5 );
	dmsg_printf("RX_ADDR_P1: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//	printf("RX_ADDR_P1: %02X\n", nrfRegRead( NRF_RX_ADDR_P1 ));
	bzero(buf, 6);
	dmsg_printf("RX_ADDR_P2: %02X\n", nrfRegRead( NRF_RX_ADDR_P2 ));
	bzero(buf, 6);
	dmsg_printf("RX_ADDR_P3: %02X\n", nrfRegRead( NRF_RX_ADDR_P3 ));
	bzero(buf, 6);
	dmsg_printf("RX_ADDR_P4: %02X\n", nrfRegRead( NRF_RX_ADDR_P4 ));
	bzero(buf, 6);
	dmsg_printf("RX_ADDR_P5: %02X\n", nrfRegRead( NRF_RX_ADDR_P5 ));
//	printf("TX_ADDR: %02X\n", nrfRegRead( NRF_TX_ADDR ));
	bzero(buf, 6);
	nrfAddrRead( NRF_TX_ADDR, buf, 5 );
	dmsg_printf("TX_ADDR: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);

//  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
	dmsg_printf("EN_AA: %02X\n", nrfRegRead( NRF_EN_AA ));
	dmsg_printf("EN_RXADDR: %02X\n", nrfRegRead( NRF_EN_RXADDR ));
	dmsg_printf("RF_CH: %02X\n", nrfRegRead( NRF_RF_CH ));
	dmsg_printf("RF_SETUP: %02X\n", nrfRegRead( NRF_RF_SETUP ));
	dmsg_printf("RX_PW_P0: %02X\n", nrfRegRead( NRF_RX_PW_P0 ));
	dmsg_printf("RX_PW_P1: %02X\n", nrfRegRead( NRF_RX_PW_P1 ));
	dmsg_printf("RX_PW_P2: %02X\n", nrfRegRead( NRF_RX_PW_P2 ));
	dmsg_printf("RX_PW_P3: %02X\n", nrfRegRead( NRF_RX_PW_P3 ));
	dmsg_printf("RX_PW_P4: %02X\n", nrfRegRead( NRF_RX_PW_P4 ));
	dmsg_printf("RX_PW_P5: %02X\n", nrfRegRead( NRF_RX_PW_P5 ));
	dmsg_printf("CONFIG: %02X\n", nrfRegRead( NRF_CONFIG ));
	dmsg_printf("CD: %02X\n", nrfRegRead( NRF_CD ));
	dmsg_printf("SETUP_AW: %02X\n", nrfRegRead( NRF_SETUP_AW ));
	dmsg_printf("SETUP_RETR: %02X\n", nrfRegRead( NRF_SETUP_RETR ));
	dmsg_printf("DYNPD: %02X\n", nrfRegRead( NRF_DYNPD ));
	dmsg_printf("FEATURE: %02X\n", nrfRegRead( NRF_FEATURE ));

	if (speed == speed_1M)
		dmsg_printf("Data Rate\t = %s\n", "1Mbps" );
	else if (speed == speed_250K)
		dmsg_printf("Data Rate\t = %s\n", "250Kbps" );
	else
		dmsg_printf("Data Rate\t = %s\n", "2Mbps" );

	dmsg_printf("Model\t\t = %s\n", "nRF24L01+"  );
	dmsg_printf("CRC Length\t = %s\n", "8 bits");
	dmsg_printf("PA Power\t = %s\n", "PA_MAX" );

}



#if 0
void setup_wifi()
{
  vTaskDelay(10 / portTICK_PERIOD_MS);
//  delay(10);
  // We start by connecting to a WiFi network
  if (verbose) {
    dbug_printf("Connecting to %s\n", ssid);
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
   vTaskDelay(500 / portTICK_PERIOD_MS);
//   delay(500);
    if (verbose) dbug_putchar('.');
  }

  if (verbose) {
    dbug_printf("\nWiFi connected: IP address: %s\n", WiFi.localIP());
  }
}

void callback(char* topic, byte* message, unsigned int length)
{
//  Serial.print("Message arrived on topic: ");
//  Serial.print(topic);
//  Serial.print(". Message: ");
}

void blink( void *pvParameters )
{
  (void) pvParameters;
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) {
    if (gError) {
      if (gError == 1) {
        dbug_puts("ERROR: Could not put item on delay queue.");
      } else if (gError == 2) {
        dbug_puts("PayLen error");
      } else {
        dbug_printf("gError: %d\n", gError);
      }
      gError = 0;
    }
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(10);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(1990);
  }
}
#endif
#if 0
/*
    Important to not set vTaskDelay to less then 10. Errors begin to develop with the MQTT and network connection.
    makes the initial wifi/mqtt connection and works to keeps those connections open.
*/
void MQTTkeepalive( void *pvParameters )
{
  // setting must be set before a mqtt connection is made
  client.setKeepAlive( 90 ); // setting keep alive to 90 seconds makes for a very reliable connection, must be set before the 1st connection is made.

  for (;;) {
    //check for a is connected and if the WiFi thinks its connected, found checking on both is more realible than just a single check
    if ( (espClient.connected()) && (WiFi.status() == WL_CONNECTED) ) {
      xSemaphoreTake( sema_MQTT_KeepAlive, portMAX_DELAY ); // whiles loop() is running no other mqtt operations should be in process
      client.loop();
      xSemaphoreGive( sema_MQTT_KeepAlive );
    } else {
      if (verbose) dbug_printf( "MQTT keep alive found MQTT status %s WiFi status %s\n", String(espClient.connected()), String(WiFi.status()) );
      if ( !(WiFi.status() == WL_CONNECTED) ) {
        connectToWiFi();
      }
      connectToMQTT();
    }
    vTaskDelay( 250 ); //task runs approx every 250 mS
  }
  vTaskDelete ( NULL );
}

void connectToMQTT()
{
  if (verbose) dbug_puts( "connect to mqtt" );
  while ( !client.connected() ) {
    client.connect("ESP8266Client");
    //client.connect( clientID, mqtt_username, mqtt_password );
    if (verbose) dbug_puts( "connecting to MQTT" );
    vTaskDelay( 250 );
  }
  if (verbose) dbug_puts("MQTT Connected");
//  client.setCallback( mqttCallback );
//  client.subscribe( mqtt_topic );
}


void connectToWiFi()
{
  byte mac[6];
  if (verbose) dbug_puts( "connect to wifi" );
  while ( WiFi.status() != WL_CONNECTED ) {
    WiFi.disconnect();
    WiFi.begin( ssid, password );
    if (verbose) dbug_puts(" waiting on wifi connection" );
    vTaskDelay( 4000 );
  }
  if (verbose) dbug_puts( "Connected to WiFi" );
  WiFi.macAddress(mac);
  if (verbose) dbug_printf( "mac address %d.%d.%d.%d.%d\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]  );
  WiFi.onEvent( WiFiEvent );
//  GetTheTime();
//  printLocalTime();
}

////
void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      dbug_puts("Connected to access point");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      dbug_puts("Disconnected from WiFi access point");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      dbug_puts("WiFi client disconnected");
      break;
    default: break;
  }
}
#endif
