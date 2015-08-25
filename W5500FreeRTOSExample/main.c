#include "stm32f10x.h"

/* low level(Hardware) initialization */
#include "IoTEVB.h"
#include "W5500HardwareDriver.h"

/* Wiznet BSD Socket library */
#include "wizchip_conf.h"
#include "socket.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>

/* Application library */
#include "dmx512.h"


#define TCP_PORT 	60000
#define UDP_PORT 	60001

#define TCP_SOCKET	0
#define UDP_SOCKET	1

#define BUFFER_SIZE	2048

DMX512 dmx;

unsigned char ucDmxBuffer[DMXBUFFERSIZE];

unsigned char tempBuffer[BUFFER_SIZE];

uint8_t mac_address[6] = {};
wiz_NetInfo gWIZNETINFO = { .mac = {},
							.ip = {222, 98, 173, 249},
							.sn = {255, 255, 255, 192},
							.gw = {222, 98, 173, 254},
							.dns = {168, 126, 63, 1},
							.dhcp = NETINFO_STATIC};

int32_t TCPServerToDMX(uint8_t sn, uint8_t* buf, uint16_t port);

static void vDMXTask( void *pvParameters );

static void EthernetToDMX( void *pvParameters );

static void countOneSecond( void *pvParameters );

int main(void)
{
	led_ctrl led1,led2;
	USART1Initialze();
		printf("USART initialized.\n\r");
	I2C1Initialize();
		printf("I2C initialized.\n\r");
	MACEEP_Read(mac_address,0xfa,6);
	led_initialize();
	/*W5500 initialization.*/
	W5500HardwareInitilize();
		printf("W5500 hardware interface initialized.\n\r");

	W5500Initialze();
		printf("W5500 IC initialized.\n\r");

	/*Set network informations*/
	wizchip_setnetinfo(&gWIZNETINFO);

	setSHAR(mac_address);

	print_network_information();

	dmx.vInit(ucDmxBuffer);

	xTaskCreate(countOneSecond, "OneSecond", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL );
	xTaskCreate(EthernetToDMX, "EthernetToDMX", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );
	xTaskCreate(vDMXTask, "DMX512", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL );
	vTaskStartScheduler();
    while(1)
    {
    	//not reach hear
    }
}

static void countOneSecond( void *pvParameters )
{
	unsigned int i;
	i = 0;
	while(1)
	{
		i++;
		vTaskDelay(1000);
		printf("%02dH:%02dM:%02dS\r\n",(int)(i/3600),(int)((i/60)%60),(int)(i%60));
	}

}

static void EthernetToDMX( void *pvParameters )
{
	while(1)
	{
		TCPServerToDMX(TCP_SOCKET,tempBuffer,TCP_PORT);
	}
}

static void vDMXTask( void *pvParameters )
{
	dmx.vInit(ucDmxBuffer);
	dmx.ucSendComplete = 1;
	while(1)
	{
		dmx.vBreak();
		dmx.vDelayUS(87);
		dmx.vIdle();
		dmx.vDelayUS(50);
		dmx.ucSendComplete = 1;
		dmx.vSendPacket();
		while(dmx.ucSendComplete);
		dmx.vDelayUS(100);
	}
}

int32_t TCPServerToDMX(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint16_t size = 0, sentsize=0;
   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED :
         if(getSn_IR(sn) & Sn_IR_CON)
         {
			setSn_IR(sn,Sn_IR_CON);
         }
		 if((size = getSn_RX_RSR(sn)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
         {
			if(size > 2048) size = 2048;
			ret = recv(sn, buf, size);
			if(ret != 512) return ret;
			else
				memcpy(ucDmxBuffer,buf,512);
         }

         break;
      case SOCK_CLOSE_WAIT :
         if((ret = disconnect(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_INIT :
         if( (ret = listen(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_CLOSED:
         if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
         break;
      default:
         break;
   }
   return 1;
}
