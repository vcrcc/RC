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
#define C_DISC 0x0B

#define ESC 0x7D

// Variáveis globais para controlo do alarme
int alarmEnabled = FALSE;
int alarmCount = 0;
LinkLayer llParams;
int txSequenceNumber = 0;
int rxSequenceNumber = 0;

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
    llParams = llParameters;
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
    txSequenceNumber = 0;
    return 1;
}

int llOpenRx(LinkLayer llParameters)
{
    llParams = llParameters;
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
    rxSequenceNumber = 0;
    return 1;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    int maxFrameSize = bufSize * 2 + 6;
    unsigned char *frame = (unsigned char *)malloc(maxFrameSize);

    frame[0] = FLAG;
    frame[1] = A_TX;
    frame[2] = txSequenceNumber == 0 ? 0x00 : 0x40;
    frame[3] = frame[1] ^ frame[2];

    unsigned char bcc2 = 0;
    int j = 4;

    for (int i = 0; i < bufSize; i++) {
        bcc2 ^= buf[i];
        if (buf[i] == FLAG || buf[i] == ESC) {
            frame[j++] = ESC;
            frame[j++] = buf[i] ^ 0x20;
        } else {
            frame[j++] = buf[i];
        }
    }

    if (bcc2 == FLAG || bcc2 == ESC) {
        frame[j++] = ESC;
        frame[j++] = bcc2 ^ 0x20;
    } else {
        frame[j++] = bcc2;
    }

    frame[j++] = FLAG;
    int frameSize = j;

    alarmCount = 0;
    alarmEnabled = FALSE;
    int success = 0;
    int reject = 0;

    unsigned char expectedRR = txSequenceNumber == 0 ? 0x85 : 0x05;
    unsigned char expectedREJ = txSequenceNumber == 0 ? 0x81 : 0x01;

    while (alarmCount < llParams.nRetransmissions && !success)
    {
        if (alarmEnabled == FALSE || reject)
        {
            writeBytesSerialPort(frame, frameSize);
            alarm(llParams.timeout);
            alarmEnabled = TRUE;
            reject = 0;
        }

        int ret = receiveTramaRRREJ(A_TX, expectedRR, expectedREJ);
        if (ret == 1) {
            alarm(0);
            success = 1;
        } else if (ret == -2) {
            alarm(0);
            reject = 1;
        }
    }

    free(frame);

    if (!success) {
        return -1;
    }

    txSequenceNumber = (txSequenceNumber + 1) % 2;
    return frameSize;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    unsigned char byte;
    State state = START;
    int dataIdx = 0;
    unsigned char expectedC = rxSequenceNumber == 0 ? 0x00 : 0x40;
    unsigned char duplicateC = rxSequenceNumber == 0 ? 0x40 : 0x00;
    
    unsigned char RR = rxSequenceNumber == 0 ? 0x85 : 0x05;
    unsigned char REJ = rxSequenceNumber == 0 ? 0x81 : 0x01;
    unsigned char RR_duplicate = rxSequenceNumber == 0 ? 0x05 : 0x85;
    
    int isDuplicate = 0;
    unsigned char bcc2 = 0;

    while (state != STOP_STATE) {
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0) {
            switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == A_TX) state = A_RCV;
                else if (byte != FLAG) state = START;
                break;
            case A_RCV:
                if (byte == expectedC) {
                    state = C_RCV;
                    isDuplicate = 0;
                } else if (byte == duplicateC) {
                    state = C_RCV;
                    isDuplicate = 1;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;
            case C_RCV:
                if (byte == (A_TX ^ (isDuplicate ? duplicateC : expectedC))) {
                    state = DATA_RCV;
                    dataIdx = 0;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;
            case DATA_RCV:
                if (byte == ESC) {
                    state = DATA_ESC_RCV;
                } else if (byte == FLAG) {
                    unsigned char receivedBcc2 = packet[dataIdx - 1];
                    bcc2 = 0;
                    for (int i = 0; i < dataIdx - 1; i++) {
                        bcc2 ^= packet[i];
                    }
                    if (bcc2 == receivedBcc2) {
                        if (isDuplicate) {
                            sendSupervisionFrame(A_TX, RR_duplicate);
                            state = START; 
                        } else {
                            sendSupervisionFrame(A_TX, RR);
                            rxSequenceNumber = (rxSequenceNumber + 1) % 2;
                            return dataIdx - 1;
                        }
                    } else {
                        if (!isDuplicate) {
                            sendSupervisionFrame(A_TX, REJ);
                        }
                        state = START;
                    }
                } else {
                    packet[dataIdx++] = byte;
                }
                break;
            case DATA_ESC_RCV:
                packet[dataIdx++] = byte ^ 0x20;
                state = DATA_RCV;
                break;
            default:
                break;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    alarmCount = 0;
    alarmEnabled = FALSE;
    int success = 0;

    while (alarmCount < llParams.nRetransmissions && !success)
    {
        if (alarmEnabled == FALSE)
        {
            sendSupervisionFrame(A_TX, C_DISC);
            alarm(llParams.timeout);
            alarmEnabled = TRUE;
        }

        if (receiveTrama(A_RX, C_DISC) == 0)
        {
            alarm(0);
            success = 1;
        }
    }

    if (!success) {
        closeSerialPort();
        return -1;
    }

    sendSupervisionFrame(A_TX, C_UA);
    sleep(1); 
    closeSerialPort();
    return 1;
}

int llCloseRx()
{
    if (receiveTrama(A_TX, C_DISC) < 0) {
        closeSerialPort();
        return -1;
    }

    alarmCount = 0;
    alarmEnabled = FALSE;
    int success = 0;

    while (alarmCount < llParams.nRetransmissions && !success)
    {
        if (alarmEnabled == FALSE)
        {
            sendSupervisionFrame(A_RX, C_DISC);
            alarm(llParams.timeout);
            alarmEnabled = TRUE;
        }

        if (receiveTrama(A_TX, C_UA) == 0)
        {
            alarm(0);
            success = 1;
        }
    }

    closeSerialPort();
    return 1;
}
