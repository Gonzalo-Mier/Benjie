/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== tcpEchoCC3100.c ========
 */

#include <string.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/drivers/GPIO.h>

#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Queue.h>

/* SimpleLink Wi-Fi Host Driver Header files */
#include <simplelink.h>

/* Example/Board Header file */
#include "Board.h"

/* Local Platform Specific Header file */
#include "sockets.h"

/* Kinematics library */
#include "Benji/Kinematics.h"

/* Port number for listening for TCP packets */
#define TCPPORT         1000

/* Spawn Task Priority */
extern const int SPAWN_TASK_PRI;

char datos[TCPPACKETSIZE];	// Buffer for receiving
int WifiON = 0;
int         bytesRcvd;
uint8_t dato_rec;
/*
 *  ======== echoFxn ========
 *  Echoes TCP messages.
 */
void echoFxn(int port)
{

	int         status;
	int         clientfd;
	int         server;
	int i;
	sockaddr_in localAddr;
	sockaddr_in clientAddr;
	socklen_t   addrlen = sizeof(clientAddr);
	char        buffer[TCPPACKETSIZE];

	server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (server == -1) {
		System_printf("Error: socket not created.\n");
		goto shutdown;
	}

	memset(&localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET;
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(port);

	status = bind(server, (const sockaddr *)&localAddr, sizeof(localAddr));
	if (status == -1) {
		System_printf("Error: bind failed.\n");
		goto shutdown;
	}

	status = listen(server, 0);
	if (status == -1){
		System_printf("Error: listen failed.\n");
		goto shutdown;
	}

	System_printf("Listo para recibir peticion.\n");
	System_flush();

	// Accept new socket
	clientfd =  accept(server, (sockaddr *)&clientAddr, &addrlen);
	//System_printf("Socket aceptado.\n"); // For debugging
	//System_flush();

	// Receive first input
	bytesRcvd = recv(clientfd, buffer, TCPPACKETSIZE, 0);


	for(i=0;i<TCPPACKETSIZE;i++)
		datos[i] = buffer[i];

	// Uninterrupt the other tasks
	WifiON = 1;
	Semaphore_post(StartSemaphore2);
	Semaphore_post(WifiSemaphore);
	Semaphore_post(ControlSemaphore);


	while(1){
		bytesRcvd = recv(clientfd, buffer, TCPPACKETSIZE, 0);

		if(bytesRcvd >= 0)
		{
			Semaphore_pend(WifiSemaphore, BIOS_WAIT_FOREVER);
			for(i=0;i<TCPPACKETSIZE;i++)
				datos[i] = buffer[i];
			dato_rec = 1;
			Semaphore_post(WifiSemaphore);
		}
		else{ // If there is an error of reception
			close(clientfd);
			// New socket
			clientfd =  accept(server, (sockaddr *)&clientAddr, &addrlen);
		}

	}

	// We won´t get here

	/* addrlen is a value-result param, must reset for next accept call */
	addrlen = sizeof(clientAddr);
	close(clientfd);


	System_printf("Error: accept failed.\n");

	shutdown:
	if (server >= 0) {
		close(server);
	}
}

/*
 *  ======== tcpEchoTask ========
 */
Void tcpEchoTask(UArg arg0, UArg arg1)
{
	void *netIF;

	/* Open WiFi and await a connection */
	netIF = socketsStartUp();

	echoFxn(TCPPORT);

	/* Close the network - don't do this if other tasks are using it */
	socketsShutDown(netIF);
}

/*
 *  ======== main ========
 */
int main(void)
{
	/* Call board init functions. */
	Board_initGeneral();
	Board_initGPIO();
	Board_initWiFi();

	/* Turn on user LED */
	GPIO_write(Board_LED0, Board_LED_ON);

	System_printf("Starting the TCP Echo example for the CC3100 \n"
			"System provider is set to SysMin. Halt the target to view"
			" any SysMin content in ROV.\n");

	/* SysMin will only print to the console when you call flush or exit */
	System_flush();

	/*
	 * The SimpleLink Host Driver requires a mechanism to allow functions to
	 * execute in temporary context.  The SpawnTask is created to handle such
	 * situations.  This task will remain blocked until the host driver
	 * posts a function.  If the SpawnTask priority is higher than other tasks,
	 * it will immediately execute the function and return to a blocked state.
	 * Otherwise, it will remain ready until it is scheduled.
	 */
	VStartSimpleLinkSpawnTask(SPAWN_TASK_PRI);

	/* Start BIOS */
	BIOS_start();

	return (0);
}
