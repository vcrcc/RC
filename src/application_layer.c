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

        unsigned char packet[MAX_PAYLOAD_SIZE];
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
        if (llOpenTx(llParameters) < 0)
        {
            fprintf(stderr, "Error establishing link layer connection (TX)\n");
            return;
        }

        // A lógica do emissor (TX) é gerida no PC do teu colega
        llCloseTx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
    }
}