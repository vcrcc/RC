// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PAYLOAD_SIZE 1000

#define CTRL_START 1
#define CTRL_DATA  2
#define CTRL_END   3

#define TYPE_FILE_SIZE 0
#define TYPE_FILE_NAME 1

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "rx") == 0)
    {
        printf("Starting Link Layer (RX)...\n");
        if (llOpenRx(llParameters) < 0)
        {
            fprintf(stderr, "Error establishing link layer connection (RX)\n");
            return;
        }

        FILE *file = fopen(filename, "wb");
        if (file == NULL)
        {
            perror("Error opening output file");
            llCloseRx();
            return;
        }

        printf("Receiver: Ready to receive file packets...\n");

        unsigned char packet[MAX_PAYLOAD_SIZE + 100];
        int bytesRead;
        long expectedFileSize = -1;
        long totalBytesWritten = 0;

        while ((bytesRead = llReceive(packet)) >= 0)
        {
            if (bytesRead == 0)
                continue;

            unsigned char controlByte = packet[0];

            // pacote start: le o tamanho esperado
            if (controlByte == CTRL_START)
            {
                int idx = 1;
                while (idx + 1 < bytesRead)
                {
                    unsigned char type = packet[idx++];
                    unsigned char length = packet[idx++];
                    if (type == TYPE_FILE_SIZE && idx + length <= bytesRead)
                    {
                        expectedFileSize = 0;
                        for (int i = 0; i < length; i++)
                        {
                            expectedFileSize = (expectedFileSize << 8) | packet[idx + i];
                        }
                    }
                    idx += length;
                }
                printf("Receiver: START control packet received. Expected file size: %ld bytes\n", expectedFileSize);
            }
            // pacote de dados: escreve no ficheiro
            else if (controlByte == CTRL_DATA)
            {
                int dataSize = 256 * packet[1] + packet[2];
                unsigned char *dataPtr = packet + 3;

                if (dataSize > 0)
                {
                    fwrite(dataPtr, 1, dataSize, file);
                    totalBytesWritten += dataSize;
                    printf("Receiver: DATA packet received (%d bytes, total written: %ld)\n", dataSize, totalBytesWritten);
                }
            }
            // pacote end: verifica se recebeu tudo
            else if (controlByte == CTRL_END)
            {
                printf("Receiver: END control packet received. Transfer complete.\n");
                if (expectedFileSize >= 0)
                {
                    if (totalBytesWritten == expectedFileSize)
                    {
                        printf("Receiver: File integrity check PASSED (%ld / %ld bytes received).\n",
                               totalBytesWritten, expectedFileSize);
                    }
                    else
                    {
                        fprintf(stderr, "Receiver: WARNING - File size mismatch! Received %ld bytes, expected %ld bytes.\n",
                                totalBytesWritten, expectedFileSize);
                    }
                }
                break;
            }
        }

        fclose(file);
        printf("Receiver: File %s created and saved successfully.\n", filename);

        if (llCloseRx() < 0)
        {
            fprintf(stderr, "Error closing link layer (RX)\n");
        }
    }
    else if (strcmp(role, "tx") == 0)
    {
        printf("Starting Link Layer (TX)...\n");
        fflush(stdout);
        if (llOpenTx(llParameters) != 0)
        {
            fprintf(stderr, "Error establishing link layer connection (TX)\n");
            return;
        }

        printf("Transmitter: Link layer established! Opening file '%s'...\n", filename);
        fflush(stdout);

        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            perror("Error opening input file");
            llCloseTx();
            return;
        }

        // calcular tamanho do ficheiro
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        rewind(file);

        printf("Transmitter: Sending file '%s' (%ld bytes)...\n", filename, fileSize);
        fflush(stdout);

        // monta pacote start com tamanho e nome do ficheiro
        unsigned char startPacket[MAX_PAYLOAD_SIZE];
        int startPacketSize = 0;
        startPacket[startPacketSize++] = CTRL_START;

        startPacket[startPacketSize++] = TYPE_FILE_SIZE;
        startPacket[startPacketSize++] = sizeof(long);
        for (int i = 0; i < (int)sizeof(long); i++)
        {
            startPacket[startPacketSize++] = (fileSize >> (8 * (sizeof(long) - 1 - i))) & 0xFF;
        }

        int nameLen = strlen(filename);
        if (nameLen > 255)
            nameLen = 255;
        startPacket[startPacketSize++] = TYPE_FILE_NAME;
        startPacket[startPacketSize++] = (unsigned char)nameLen;
        memcpy(startPacket + startPacketSize, filename, nameLen);
        startPacketSize += nameLen;

        if (llSend(startPacket, startPacketSize) < 0)
        {
            fprintf(stderr, "Error sending START control packet\n");
            fclose(file);
            llCloseTx();
            return;
        }
        printf("Transmitter: START control packet sent.\n");

        // envia os dados divididos em pacotes
        int maxChunkSize = MAX_PAYLOAD_SIZE - 3;
        unsigned char dataBuffer[maxChunkSize];
        unsigned char dataPacket[MAX_PAYLOAD_SIZE];
        int packetCount = 0;
        int bytesRead;

        while ((bytesRead = fread(dataBuffer, 1, maxChunkSize, file)) > 0)
        {
            dataPacket[0] = CTRL_DATA;
            dataPacket[1] = (bytesRead >> 8) & 0xFF;
            dataPacket[2] = bytesRead & 0xFF;
            memcpy(dataPacket + 3, dataBuffer, bytesRead);

            if (llSend(dataPacket, bytesRead + 3) < 0)
            {
                fprintf(stderr, "Error sending DATA packet #%d\n", packetCount);
                fclose(file);
                llCloseTx();
                return;
            }
            printf("Transmitter: Sent DATA packet #%d (%d bytes)\n", packetCount, bytesRead);
            packetCount++;
        }

        fclose(file);

        // pacote end com a mesma informacao do start
        unsigned char endPacket[MAX_PAYLOAD_SIZE];
        int endPacketSize = 0;
        endPacket[endPacketSize++] = CTRL_END;

        endPacket[endPacketSize++] = TYPE_FILE_SIZE;
        endPacket[endPacketSize++] = sizeof(long);
        for (int i = 0; i < (int)sizeof(long); i++)
        {
            endPacket[endPacketSize++] = (fileSize >> (8 * (sizeof(long) - 1 - i))) & 0xFF;
        }

        endPacket[endPacketSize++] = TYPE_FILE_NAME;
        endPacket[endPacketSize++] = (unsigned char)nameLen;
        memcpy(endPacket + endPacketSize, filename, nameLen);
        endPacketSize += nameLen;

        if (llSend(endPacket, endPacketSize) < 0)
        {
            fprintf(stderr, "Error sending END control packet\n");
            llCloseTx();
            return;
        }
        printf("Transmitter: END control packet sent. Transfer complete.\n");

        if (llCloseTx() < 0)
        {
            fprintf(stderr, "Error closing link layer (TX)\n");
            return;
        }
        printf("Transmitter: Connection closed successfully.\n");
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
    }
}