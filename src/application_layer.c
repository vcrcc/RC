// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PAYLOAD_SIZE 1000

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

        while ((bytesRead = llReceive(packet)) >= 0)
        {
            if (bytesRead == 0)
                continue;

            unsigned char controlByte = packet[0];

            if (controlByte == 2) // Pacote de Controlo START
            {
                printf("Receiver: START control packet received.\n");
            }
            else if (controlByte == 1) // Pacote de DADOS
            {
                // packet[0] = 1 (Campo de controlo)
                // packet[1] = Número de sequência
                // packet[2] = L2 (Byte alto do tamanho)
                // packet[3] = L1 (Byte baixo do tamanho)
                int dataSize = 256 * packet[2] + packet[3];

                fwrite(packet + 4, 1, dataSize, file);
                printf("Receiver: DATA packet received (%d bytes, sequence %d)\n", dataSize, packet[1]);
            }
            else if (controlByte == 3) // Pacote de Controlo END
            {
                printf("Receiver: END control packet received. Transfer complete.\n");
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

        // Determinar o tamanho do ficheiro
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        rewind(file);

        printf("Transmitter: Sending file '%s' (%ld bytes)...\n", filename, fileSize);
        fflush(stdout);

        // Construir e enviar pacote de controlo START (C = 2)
        // Com TLV: T=0 (tamanho do ficheiro), L, V
        unsigned char startPacket[MAX_PAYLOAD_SIZE];
        startPacket[0] = 2; // START
        startPacket[1] = 0; // T: tamanho
        startPacket[2] = sizeof(long); // L
        for (int i = 0; i < (int)sizeof(long); i++)
        {
            startPacket[3 + i] = (fileSize >> (8 * (sizeof(long) - 1 - i))) & 0xFF;
        }
        int startPacketSize = 3 + sizeof(long);

        if (llSend(startPacket, startPacketSize) < 0)
        {
            fprintf(stderr, "Error sending START control packet\n");
            fclose(file);
            llCloseTx();
            return;
        }
        printf("Transmitter: START control packet sent.\n");

        // Ler e enviar pacotes de DADOS (C = 1)
        // [ C=1 | sequenceNumber | L2 | L1 | dados... ]
        int maxChunkSize = MAX_PAYLOAD_SIZE - 4;
        unsigned char dataBuffer[maxChunkSize];
        unsigned char dataPacket[MAX_PAYLOAD_SIZE];
        int sequenceNumber = 0;
        int bytesRead;

        while ((bytesRead = fread(dataBuffer, 1, maxChunkSize, file)) > 0)
        {
            dataPacket[0] = 1; // DATA
            dataPacket[1] = sequenceNumber % 255;
            dataPacket[2] = (bytesRead >> 8) & 0xFF; // L2
            dataPacket[3] = bytesRead & 0xFF;        // L1
            memcpy(dataPacket + 4, dataBuffer, bytesRead);

            if (llSend(dataPacket, bytesRead + 4) < 0)
            {
                fprintf(stderr, "Error sending DATA packet (sequence %d)\n", sequenceNumber);
                fclose(file);
                llCloseTx();
                return;
            }
            printf("Transmitter: Sent DATA packet (sequence %d, %d bytes)\n", sequenceNumber, bytesRead);
            sequenceNumber++;
        }

        fclose(file);

        // Construir e enviar pacote de controlo END (C = 3)
        unsigned char endPacket[MAX_PAYLOAD_SIZE];
        endPacket[0] = 3; // END
        endPacket[1] = 0; // T: tamanho
        endPacket[2] = sizeof(long); // L
        for (int i = 0; i < (int)sizeof(long); i++)
        {
            endPacket[3 + i] = (fileSize >> (8 * (sizeof(long) - 1 - i))) & 0xFF;
        }
        int endPacketSize = 3 + sizeof(long);

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