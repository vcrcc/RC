// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CTRL_START 1
#define CTRL_DATA  2
#define CTRL_END   3

#define TYPE_FILE_SIZE 0
#define TYPE_FILE_NAME 1

typedef enum
{
    APP_WAIT_START,
    APP_RECEIVE_DATA,
    APP_DONE,
    APP_ERROR
} AppRxState;

static int parseControlPacket(const unsigned char *packet, int packetSize,
                              long *fileSizeOut, char *fileNameOut,
                              size_t fileNameMaxLen, int *hasFileNameOut)
{
    if (packet == NULL || fileSizeOut == NULL || packetSize < 3)
    {
        return -1;
    }

    int hasFileSize = 0;
    int hasFileName = 0;
    if (hasFileNameOut != NULL)
    {
        *hasFileNameOut = 0;
    }
    if (fileNameOut != NULL && fileNameMaxLen > 0)
    {
        fileNameOut[0] = '\0';
    }

    int idx = 1;
    while (idx < packetSize)
    {
        if (idx + 2 > packetSize)
        {
            return -1; // TLV incompleto
        }

        unsigned char type = packet[idx++];
        unsigned char length = packet[idx++];

        if (idx + length > packetSize)
        {
            return -1; // Valor TLV truncado
        }

        if (type == TYPE_FILE_SIZE)
        {
            if (hasFileSize || length == 0 || length > (int)sizeof(unsigned long))
            {
                return -1;
            }
            unsigned long parsedSize = 0;
            for (int i = 0; i < length; i++)
            {
                parsedSize = (parsedSize << 8) | packet[idx + i];
            }
            if ((long)parsedSize < 0)
            {
                return -1;
            }
            *fileSizeOut = (long)parsedSize;
            hasFileSize = 1;
        }
        else if (type == TYPE_FILE_NAME)
        {
            if (hasFileName || length == 0)
            {
                return -1;
            }
            if (fileNameOut != NULL && fileNameMaxLen > 0)
            {
                size_t copyLen = (length < fileNameMaxLen - 1) ? length : (fileNameMaxLen - 1);
                memcpy(fileNameOut, packet + idx, copyLen);
                fileNameOut[copyLen] = '\0';
            }
            hasFileName = 1;
            if (hasFileNameOut != NULL)
            {
                *hasFileNameOut = 1;
            }
        }

        idx += length;
    }

    if (!hasFileSize)
    {
        return -1;
    }

    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    if (serialPort == NULL || role == NULL || filename == NULL)
    {
        fprintf(stderr, "Error: Invalid NULL arguments to applicationLayer.\n");
        return;
    }

    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    int portLen = snprintf(llParameters.serialPort, sizeof(llParameters.serialPort), "%s", serialPort);
    if (portLen < 0 || (size_t)portLen >= sizeof(llParameters.serialPort))
    {
        fprintf(stderr, "Error: Serial port path '%s' exceeds maximum length (%zu).\n",
                serialPort, sizeof(llParameters.serialPort) - 1);
        return;
    }

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

        unsigned char packet[MAX_PAYLOAD_SIZE];
        int bytesRead = 0;
        long expectedFileSize = -1;
        long totalBytesWritten = 0;
        char startFileName[256] = {0};
        int hasStartFileName = 0;
        AppRxState appState = APP_WAIT_START;

        while ((bytesRead = llReceive(packet)) >= 0)
        {
            if (bytesRead == 0)
                continue;

            unsigned char controlByte = packet[0];

            // pacote start: le e valida o tamanho esperado
            if (controlByte == CTRL_START)
            {
                if (appState != APP_WAIT_START)
                {
                    fprintf(stderr, "Receiver: Error - Unexpected START packet received.\n");
                    appState = APP_ERROR;
                    break;
                }

                if (parseControlPacket(packet, bytesRead, &expectedFileSize,
                                       startFileName, sizeof(startFileName),
                                       &hasStartFileName) < 0)
                {
                    fprintf(stderr, "Receiver: Error - Malformed START control packet.\n");
                    appState = APP_ERROR;
                    break;
                }

                appState = APP_RECEIVE_DATA;
                printf("Receiver: START control packet received. Expected file size: %ld bytes\n", expectedFileSize);
            }
            // pacote de dados: valida tamanho e escreve no ficheiro
            else if (controlByte == CTRL_DATA)
            {
                if (appState != APP_RECEIVE_DATA)
                {
                    fprintf(stderr, "Receiver: Error - DATA packet received before START.\n");
                    appState = APP_ERROR;
                    break;
                }

                if (bytesRead < 3)
                {
                    fprintf(stderr, "Receiver: Error - DATA packet too short (%d bytes).\n", bytesRead);
                    appState = APP_ERROR;
                    break;
                }

                int dataSize = 256 * packet[1] + packet[2];
                if (dataSize != bytesRead - 3 || dataSize > MAX_PAYLOAD_SIZE - 3)
                {
                    fprintf(stderr, "Receiver: Error - Invalid DATA length field (%d vs %d payload bytes).\n",
                            dataSize, bytesRead - 3);
                    appState = APP_ERROR;
                    break;
                }

                if (totalBytesWritten + dataSize > expectedFileSize)
                {
                    fprintf(stderr, "Receiver: Error - Received data exceeds expected file size (%ld > %ld).\n",
                            totalBytesWritten + dataSize, expectedFileSize);
                    appState = APP_ERROR;
                    break;
                }

                unsigned char *dataPtr = packet + 3;

                if (dataSize > 0)
                {
                    size_t written = fwrite(dataPtr, 1, (size_t)dataSize, file);
                    if (written != (size_t)dataSize || fflush(file) != 0)
                    {
                        perror("Receiver: Error writing to output file");
                        appState = APP_ERROR;
                        break;
                    }
                    totalBytesWritten += dataSize;
                    printf("Receiver: DATA packet received (%d bytes, total written: %ld)\n", dataSize, totalBytesWritten);
                }
            }
            // pacote end: verifica correspondencia com o start e integridade do tamanho
            else if (controlByte == CTRL_END)
            {
                if (appState != APP_RECEIVE_DATA)
                {
                    fprintf(stderr, "Receiver: Error - END packet received before START.\n");
                    appState = APP_ERROR;
                    break;
                }

                long endFileSize = -1;
                char endFileName[256] = {0};
                int hasEndFileName = 0;

                if (parseControlPacket(packet, bytesRead, &endFileSize,
                                       endFileName, sizeof(endFileName),
                                       &hasEndFileName) < 0)
                {
                    fprintf(stderr, "Receiver: Error - Malformed END control packet.\n");
                    appState = APP_ERROR;
                    break;
                }

                if (endFileSize != expectedFileSize ||
                    hasEndFileName != hasStartFileName ||
                    (hasStartFileName && strcmp(startFileName, endFileName) != 0))
                {
                    fprintf(stderr, "Receiver: Error - END packet does not match START packet.\n");
                    appState = APP_ERROR;
                    break;
                }

                printf("Receiver: END control packet received. Transfer complete.\n");
                if (totalBytesWritten != expectedFileSize)
                {
                    fprintf(stderr, "Receiver: Error - File size mismatch! Received %ld bytes, expected %ld bytes.\n",
                            totalBytesWritten, expectedFileSize);
                    appState = APP_ERROR;
                    break;
                }

                appState = APP_DONE;
                break;
            }
            else
            {
                fprintf(stderr, "Receiver: Error - Invalid control byte (0x%02X).\n", controlByte);
                appState = APP_ERROR;
                break;
            }
        }

        if (bytesRead < 0)
        {
            fprintf(stderr, "Receiver: Error receiving packet from link layer.\n");
            appState = APP_ERROR;
        }

        if (fclose(file) != 0)
        {
            perror("Receiver: Error closing output file");
            appState = APP_ERROR;
        }

        if (appState == APP_DONE)
        {
            printf("Receiver: File integrity check PASSED (%ld / %ld bytes received).\n",
                   totalBytesWritten, expectedFileSize);
            printf("Receiver: File %s created and saved successfully.\n", filename);
        }
        else
        {
            fprintf(stderr, "Receiver: File transfer failed.\n");
        }

        if (llCloseRx() < 0)
        {
            fprintf(stderr, "Error closing link layer (RX)\n");
        }
    }
    else if (strcmp(role, "tx") == 0)
    {
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            perror("Error opening input file");
            return;
        }

        // calcular tamanho do ficheiro antes de abrir a ligacao
        if (fseek(file, 0, SEEK_END) != 0)
        {
            perror("Error seeking input file");
            fclose(file);
            return;
        }
        long fileSize = ftell(file);
        if (fileSize < 0)
        {
            perror("Error getting input file size");
            fclose(file);
            return;
        }
        rewind(file);

        int nameLen = (int)strlen(filename);
        if (nameLen > 255)
            nameLen = 255;

        int controlPacketSize = 1 + 2 + (int)sizeof(long) + 2 + nameLen;
        if (controlPacketSize > MAX_PAYLOAD_SIZE || MAX_PAYLOAD_SIZE <= 3)
        {
            fprintf(stderr, "Error: MAX_PAYLOAD_SIZE (%d) is too small for control packet (%d bytes).\n",
                    MAX_PAYLOAD_SIZE, controlPacketSize);
            fclose(file);
            return;
        }

        printf("Starting Link Layer (TX)...\n");
        fflush(stdout);
        if (llOpenTx(llParameters) != 0)
        {
            fprintf(stderr, "Error establishing link layer connection (TX)\n");
            fclose(file);
            return;
        }

        printf("Transmitter: Link layer established! Sending file '%s' (%ld bytes)...\n", filename, fileSize);
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

        if (ferror(file))
        {
            perror("Error reading from input file");
            fclose(file);
            llCloseTx();
            return;
        }

        if (fclose(file) != 0)
        {
            perror("Error closing input file");
            llCloseTx();
            return;
        }

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