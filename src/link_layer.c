// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01
#define C_SET 0x03
#define C_UA 0x07

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

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

int receiveSupervisionFrame(unsigned char expectedA, unsigned char expectedC)
{
    State state = START;
    unsigned char byte = 0;

    while (state != STOP_STATE)
    {
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0)
        {
            printf("Byte received: 0x%02X\n", (unsigned int)(byte & 0xFF));

            switch (state)
            {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == expectedA)
                    state = A_RCV;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case A_RCV:
                if (byte == expectedC)
                    state = C_RCV;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case C_RCV:
                if (byte == (expectedA ^ expectedC))
                    state = BCC_OK;
                else if (byte == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;
            case BCC_OK:
                if (byte == FLAG)
                    state = STOP_STATE;
                else
                    state = START;
                break;
            default:
                state = START;
                break;
            }
        }
    }
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and send a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Enviar trama SET
    printf("Transmitter: Sending SET frame...\n");
    if (sendSupervisionFrame(A_TX, C_SET) < 0)
    {
        perror("sendSupervisionFrame SET");
        closeSerialPort();
        return -1;
    }

    // Aguardar trama UA
    printf("Transmitter: Waiting for UA frame...\n");
    if (receiveSupervisionFrame(A_TX, C_UA) < 0)
    {
        fprintf(stderr, "Error receiving UA frame\n");
        closeSerialPort();
        return -1;
    }

    printf("Connection successfully established! (UA received)\n");

    // Close serial port
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
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Aguardar trama SET
    printf("Receiver: Waiting for SET frame...\n");
    if (receiveSupervisionFrame(A_TX, C_SET) < 0)
    {
        fprintf(stderr, "Error receiving SET frame\n");
        closeSerialPort();
        return -1;
    }

    // Responder com trama UA
    printf("Receiver: Sending UA frame...\n");
    if (sendSupervisionFrame(A_TX, C_UA) < 0)
    {
        perror("sendSupervisionFrame UA");
        closeSerialPort();
        return -1;
    }

    printf("Connection successfully established! (SET received, UA sent)\n");

    // Close serial port
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
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}