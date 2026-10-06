// RCOM 2026/2027
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "state_machine.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01
#define C_SET 0x03
#define C_UA 0x07

// Variáveis globais para controlo do alarme
int alarmEnabled = FALSE;
int alarmCount = 0;

// Handler para o sinal SIGALRM
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}



int sendSupervisionFrame(unsigned char address, unsigned char control)
{
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;
    frame[4] = FLAG;

    printf("Sending frame: ");
    for (int i = 0; i < 5; i++)
    {
        printf("0x%02X ", (unsigned int)(frame[i] & 0xFF));
    }
    printf("\n");

    return writeBytesSerialPort(frame, 5);
}



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (TX)\n", llParameters.serialPort);

    // Configuração do sinal de alarme com sigaction
    struct sigaction act = {0};
    act.sa_handler = alarmHandler;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;

    if (sigaction(SIGALRM, &act, NULL) < 0)
    {
        perror("sigaction");
        closeSerialPort();
        return -1;
    }

    alarmCount = 0;
    alarmEnabled = FALSE;
    int success = 0;

    // Ciclo de envio e retransmissões por timeout
    while (alarmCount < llParameters.nRetransmissions && !success)
    {
        if (alarmEnabled == FALSE)
        {
            printf("Transmitter: Sending SET frame (attempt %d)...\n", alarmCount + 1);
            if (sendSupervisionFrame(A_TX, C_SET) < 0)
            {
                perror("sendSupervisionFrame SET");
                closeSerialPort();
                return -1;
            }

            alarm(llParameters.timeout); // Ativa o alarme
            alarmEnabled = TRUE;
        }

        if (receiveTrama(A_TX, C_UA) == 0)
        {
            alarm(0); // Desativa o alarme pendente ao receber UA
            success = 1;
        }
    }

    if (!success)
    {
        fprintf(stderr, "Error: Exceeded maximum number of retransmissions (%d)\n", llParameters.nRetransmissions);
        closeSerialPort();
        return -1;
    }

    printf("Connection successfully established! (UA received)\n");

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (RX)\n", llParameters.serialPort);

    printf("Receiver: Waiting for SET frame...\n");
    if (receiveTrama(A_TX, C_SET) < 0)
    {
        fprintf(stderr, "Error receiving SET frame\n");
        closeSerialPort();
        return -1;
    }

    printf("Receiver: Sending UA frame...\n");
    if (sendSupervisionFrame(A_TX, C_UA) < 0)
    {
        perror("sendSupervisionFrame UA");
        closeSerialPort();
        return -1;
    }

    printf("Connection successfully established! (SET received, UA sent)\n");

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    return 0;
}

int llCloseRx()
{
    return 0;
}