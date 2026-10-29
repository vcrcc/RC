// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256
#define FLAG 0x7E
#define C_SET 0x03
#define C_UA 0x07
#define A_TX 0x03
#define A_RX 0x01
#define ESC 0x7D
#define ESC_XOR 0x20
#define C_I_0 0x00
#define C_I_1 0x80
#define C_RR_0 0xAA
#define C_RR_1 0xAB
#define C_REJ_0 0x54
#define C_REJ_1 0x55
#define C_DISC 0x0B

static int alarmEnabled = FALSE;
static int alarmCount = 0;

static LinkLayer parameters;
static int txSequenceNumber = 0;
static int rxSequenceNumber = 0;

// estatisticas de envio e rececao
static int statsFramesSent = 0;
static int statsFramesReceived = 0;
static int statsTimeouts = 0;
static int statsRejSent = 0;
static int statsRejReceived = 0;
static int statsDuplicates = 0;

// trata o alarme sigalrm
static void alarmHandler(int signal) {
  (void)signal;
  alarmEnabled = FALSE;
  alarmCount++;
  statsTimeouts++;

  printf("Alarm #%d received\n", alarmCount);
}

typedef enum {
  START,
  FLAG_RCV,
  ADRESS_RCV,
  CONTROL_RCV,
  BCC_OK,
  DATA_RCV,
  STOP
} States;

static void updateStateMachine(States *state, unsigned char byte,
                               unsigned char expectedA, unsigned char *control,
                               unsigned char *dataBuffer, int *dataIndex) {
  static int isEscaped = FALSE;

  switch (*state) {
  case START:
    isEscaped = FALSE;
    if (byte == FLAG) {
      *state = FLAG_RCV;
    }
    break;

  case FLAG_RCV:
    if (byte == expectedA) {
      *state = ADRESS_RCV;
    } else if (byte == FLAG) {
      *state = FLAG_RCV;
    } else {
      *state = START;
    }
    break;

  case ADRESS_RCV:
    if (byte == FLAG) {
      *state = FLAG_RCV;
    } else {
      if (control != NULL) {
        *control = byte;
      }
      *state = CONTROL_RCV;
    }
    break;

  case CONTROL_RCV:
    if (control != NULL && byte == (expectedA ^ (*control))) {
      if (dataBuffer != NULL && dataIndex != NULL) {
        *state = DATA_RCV;
        *dataIndex = 0;
        isEscaped = FALSE;
      } else {
        *state = BCC_OK;
      }
    } else if (byte == FLAG) {
      *state = FLAG_RCV;
    } else {
      *state = START;
    }
    break;

  case BCC_OK:
    if (byte == FLAG) {
      *state = STOP;
    } else {
      *state = START;
    }
    break;

  case DATA_RCV:
    if (dataBuffer != NULL && dataIndex != NULL) {
      if (byte == ESC && !isEscaped) {
        isEscaped = TRUE;
      } else if (isEscaped) {
        if (*dataIndex < MAX_PAYLOAD_SIZE + 10) {
          dataBuffer[(*dataIndex)++] = byte ^ ESC_XOR;
        }
        isEscaped = FALSE;
      } else if (byte == FLAG) {
        *state = STOP;
      } else {
        if (*dataIndex < MAX_PAYLOAD_SIZE + 10) {
          dataBuffer[(*dataIndex)++] = byte;
        } else {
          // Trama excedeu o tamanho maximo por causa de ruido
          *state = START;
        }
      }
    } else {
      *state = START;
    }
    break;

  case STOP:
    break;
  }
}

// Função auxiliar de enviar a trama de 5 bytes, feita para reutilizar o código
static int sendTrama(unsigned char address, unsigned char control) {
  unsigned char frame[5];
  frame[0] = FLAG;
  frame[1] = address;
  frame[2] = control;
  frame[3] = address ^ control;
  frame[4] = FLAG;

  return writeBytesSerialPort(frame, 5);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters) {

  parameters = llParameters;
  txSequenceNumber = 0;
  statsFramesSent = 0;
  statsTimeouts = 0;
  statsRejReceived = 0;

  if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
    perror("openSerialPort");
    return -1;
  }

  printf("Serial port %s opened\n", llParameters.serialPort);

  struct sigaction act = {0};
  act.sa_handler = &alarmHandler;
  if (sigaction(SIGALRM, &act, NULL) == -1) {
    perror("sigaction");
    return -1;
  }

  alarmCount = 0;
  alarmEnabled = FALSE;

  States state = START;
  unsigned char receivedC = 0;

  while (alarmCount < llParameters.nRetransmissions && state != STOP) {
    if (alarmEnabled == FALSE) {
      int bytes = sendTrama(A_TX, C_SET);
      printf("%d bytes written (SET frame sent, attempt #%d)\n", bytes,
             alarmCount + 1);
      alarm(llParameters.timeout);
      alarmEnabled = TRUE;
      state = START; // incializa a state machine sempre que inicia o alarme
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, &receivedC, NULL, NULL);
      if (state == STOP && receivedC != C_UA) {
        state = START;
      }
    }
  }

  if (state == STOP) {
    alarm(0);
    printf("UA frame successfully received and verified via State Machine!\n");
    return 0;
  }

  fprintf(stderr, "Error: Connection timed out after %d attempts\n",
          llParameters.nRetransmissions);
  return -1;
}

int llOpenRx(LinkLayer llParameters) {

  parameters = llParameters;
  rxSequenceNumber = 0;
  statsFramesReceived = 0;
  statsDuplicates = 0;
  statsRejSent = 0;

  if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
    perror("openSerialPort");
    return -1;
  }

  printf("Serial port %s opened\n", llParameters.serialPort);

  States state = START;
  unsigned char receivedC = 0;

  while (state != STOP) {
    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, &receivedC, NULL, NULL);
      if (state == STOP && receivedC != C_SET) {
        state = START;
      }
    }
  }

  printf("SET frame successfully received and verified via State Machine!\n");

  int bytes = sendTrama(A_TX, C_UA);
  printf("%d bytes written (UA frame sent)\n", bytes);

  return 0;
}

static int stuffed_trama_tamanho(unsigned char *buf_destino,
                                 const unsigned char *buf_fonte, int bufSize) {
  buf_destino[0] = FLAG;
  buf_destino[1] = A_TX;
  if (txSequenceNumber == 0) {
    buf_destino[2] = C_I_0;
  } else if (txSequenceNumber == 1) {
    buf_destino[2] = C_I_1;
  }
  buf_destino[3] = buf_destino[1] ^ buf_destino[2];
  int index = 4;
  unsigned char bcc2 = 0;
  for (int i = 0; i < bufSize; i++) {
    bcc2 ^= buf_fonte[i];
    if (buf_fonte[i] == FLAG || buf_fonte[i] == ESC) {
      buf_destino[index++] = ESC;
      buf_destino[index++] = buf_fonte[i] ^ ESC_XOR;
    } else {
      buf_destino[index++] = buf_fonte[i];
    }
  }
  if (bcc2 == FLAG || bcc2 == ESC) {
    buf_destino[index++] = ESC;
    buf_destino[index++] = bcc2 ^ ESC_XOR;
  } else {
    buf_destino[index++] = bcc2;
  }
  buf_destino[index++] = FLAG; // enerrar o buf e retornar size
  return index;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize) {
  // A app pode enviar até MAX_PAYLOAD_SIZE bytes, o dobro pelo stuffing, +10
  // para cobrir FLAG, Address, Control,bcc1 e BCC2 (pode ser stuffed) e FLAG
  unsigned char trama[2 * MAX_PAYLOAD_SIZE + 10];
  int tramaSize = stuffed_trama_tamanho(trama, buf, bufSize);

  struct sigaction act = {0};
  act.sa_handler = &alarmHandler;
  if (sigaction(SIGALRM, &act, NULL) == -1) {
    perror("sigaction");
    return -1;
  }

  alarmCount = 0;
  alarmEnabled = FALSE;

  unsigned char expectedRR = (txSequenceNumber == 0) ? C_RR_1 : C_RR_0;
  unsigned char expectedREJ = (txSequenceNumber == 0) ? C_REJ_0 : C_REJ_1;

  States state = START;
  unsigned char receivedC = 0;

  while (alarmCount < parameters.nRetransmissions) {
    if (alarmEnabled == FALSE) {
      int bytes = writeBytesSerialPort(trama, tramaSize);
      statsFramesSent++;
      printf("%d bytes written (I frame sent, seq %d, attempt #%d)\n", bytes,
             txSequenceNumber, alarmCount + 1);
      alarm(parameters.timeout);
      alarmEnabled = TRUE;
      state = START;
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, &receivedC, NULL, NULL);

      if (state == STOP) {
        if (receivedC == expectedRR) {
          alarm(0);
          printf("Positive ACK (RR) received!\n");
          txSequenceNumber ^= 1; // Alterna a sequência (0 -> 1 ou 1 -> 0)
          return tramaSize;
        } else if (receivedC == expectedREJ) {
          alarm(0);
          statsRejReceived++;
          printf(
              "Negative ACK (REJ) received! Retransmitting immediately...\n");
          alarmEnabled = FALSE;
          alarmCount++; // conta a tentativa de retransmissao
          state = START;
        } else {
          // Outro byte de controlo inesperado: ignora e recomeça
          state = START;
        }
      }
    }
  }

  fprintf(stderr, "Error: llSend timed out after %d attempts\n",
          parameters.nRetransmissions);
  return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////

int llReceive(unsigned char *packet) {
  unsigned char destuffed[MAX_PAYLOAD_SIZE + 10];
  int index = 0;
  States state = START;
  unsigned char receivedC = 0;

  // ler da porta serie ate fechar a trama
  while (state != STOP) {
    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, &receivedC, destuffed, &index);
    }
  }

  if (index < 2) {
    return 0; // trama vazia ou ruido: ignora e deixa continuar
  }

  // o ultimo byte recebido e o bcc2, o resto sao os dados
  int dataSize = index - 1;
  unsigned char receivedBcc2 = destuffed[index - 1];

  // calcular bcc2 dos dados recebidos para comparar
  unsigned char calculatedBcc2 = 0;
  for (int i = 0; i < dataSize; i++) {
    calculatedBcc2 ^= destuffed[i];
  }

  // se o bcc2 calculado der diferente, houve erro nos dados e manda rej
  if (calculatedBcc2 != receivedBcc2) {
    statsRejSent++;
    unsigned char rej = (receivedC == C_I_0) ? C_REJ_0 : C_REJ_1;
    sendTrama(A_TX, rej);
    printf("BCC2 error detected! REJ frame sent\n");
    return 0;
  }

  // dados corretos: verificar se e nova ou repetida
  unsigned char expectedC = (rxSequenceNumber == 0) ? C_I_0 : C_I_1;

  if (receivedC == expectedC) {
    // trama certa: copia dados para o packet, alterna a sequencia e manda rr
    for (int i = 0; i < dataSize; i++) {
      packet[i] = destuffed[i];
    }
    rxSequenceNumber ^= 1;
    statsFramesReceived++;
    unsigned char rr = (rxSequenceNumber == 0) ? C_RR_0 : C_RR_1;
    sendTrama(A_TX, rr);
    printf("Frame accepted (seq %d)! RR frame sent\n",
           (receivedC == C_I_0) ? 0 : 1);
    return dataSize;
  } else {
    // se for duplicada descarta os dados mas responde rr para destravar o tx
    statsDuplicates++;
    unsigned char rr = (rxSequenceNumber == 0) ? C_RR_0 : C_RR_1;
    sendTrama(A_TX, rr);
    printf("Duplicate frame received! Resent RR\n");
    return 0;
  }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx() {
  struct sigaction act = {0};
  act.sa_handler = &alarmHandler;
  if (sigaction(SIGALRM, &act, NULL) == -1) {
    perror("sigaction");
    return -1;
  }

  alarmCount = 0;
  alarmEnabled = FALSE;

  States state = START;
  unsigned char receivedC = 0;

  // envia disc e espera por disc do rx com alarmes
  while (alarmCount < parameters.nRetransmissions && state != STOP) {
    if (alarmEnabled == FALSE) {
      int bytes = sendTrama(A_TX, C_DISC);
      printf("%d bytes written (DISC frame sent, attempt #%d)\n", bytes,
             alarmCount + 1);
      alarm(parameters.timeout);
      alarmEnabled = TRUE;
      state = START;
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_RX, &receivedC, NULL, NULL);
      if (state == STOP && receivedC != C_DISC) {
        state = START;
      }
    }
  }

  if (state == STOP) {
    alarm(0);
    printf("DISC frame successfully received from RX!\n");

    // responde com ua para fechar a ligacao de vez
    sendTrama(A_RX, C_UA);
    printf("UA frame sent to RX\n");

    printf("\n--- Estatisticas TX ---\n");
    printf("Tramas I enviadas: %d\n", statsFramesSent);
    printf("Timeouts: %d\n", statsTimeouts);
    printf("REJ recebidos: %d\n", statsRejReceived);
    printf("-----------------------\n\n");

    sleep(1); // dar tempo para o rx ler o ua antes de fechar a porta
    if (closeSerialPort() < 0) {
      return -1;
    }
    return 0;
  }

  fprintf(stderr, "Error: Connection close timed out\n");
  closeSerialPort();
  return -1;
}

int llCloseRx() {
  States state = START;
  unsigned char receivedC = 0;

  // espera pelo disc enviado pelo emissor
  while (state != STOP) {
    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, &receivedC, NULL, NULL);
      if (state == STOP && receivedC != C_DISC) {
        state = START;
      }
    }
  }

  printf("DISC frame received from TX!\n");

  // responde com disc com endereco A_RX
  sendTrama(A_RX, C_DISC);
  printf("DISC frame sent back to TX\n");

  // espera pela confirmacao final com ua
  state = START;
  while (state != STOP) {
    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_RX, &receivedC, NULL, NULL);
      if (state == STOP && receivedC != C_UA) {
        state = START;
      }
    }
  }

  printf("UA frame received from TX! Connection closed.\n");

  printf("\n--- Estatisticas RX ---\n");
  printf("Tramas I recebidas: %d\n", statsFramesReceived);
  printf("Tramas repetidas: %d\n", statsDuplicates);
  printf("REJ enviados: %d\n", statsRejSent);
  printf("-----------------------\n\n");

  if (closeSerialPort() < 0) {
    return -1;
  }

  return 0;
}
