// RCOM 2026/2027
//
// Link layer protocol implementation
#define _POSIX_SOURCE 1 // POSIX compliant source

#include "link_layer.h"
#include "serial_port.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// MISC
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

static volatile sig_atomic_t alarmEnabled = FALSE;
static volatile sig_atomic_t alarmTriggered = FALSE;
static int alarmCount = 0;

static LinkLayer parameters;
static int txSequenceNumber = 0;
static int rxSequenceNumber = 0;
static int dataFieldError = FALSE;
static int rxDiscReceived = FALSE;

// estatisticas de envio e rececao
static int statsFramesSent = 0;
static int statsFramesReceived = 0;
static int statsTimeouts = 0;
static int statsRejSent = 0;
static int statsRejReceived = 0;
static int statsDuplicates = 0;

// trata o alarme sigalrm (apenas operacoes async-signal-safe)
static void alarmHandler(int signal) {
  (void)signal;
  alarmEnabled = FALSE;
  alarmTriggered = TRUE;
}

static void processAlarmTrigger(void) {
  if (alarmTriggered) {
    alarmTriggered = FALSE;
    alarmCount++;
    statsTimeouts++;
    printf("Alarm #%d received\n", alarmCount);
  }
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
  static unsigned char currentA = 0;

  switch (*state) {
  case START:
    isEscaped = FALSE;
    dataFieldError = FALSE;
    if (byte == FLAG) {
      *state = FLAG_RCV;
    }
    break;

  case FLAG_RCV:
    isEscaped = FALSE;
    dataFieldError = FALSE;
    if ((expectedA != 0 && byte == expectedA) ||
        (expectedA == 0 && (byte == A_TX || byte == A_RX))) {
      currentA = byte;
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
    } else if (expectedA == 0 &&
               !((currentA == A_RX && byte == C_UA) ||
                 (currentA == A_TX && byte == C_DISC))) {
      *state = START;
    } else {
      if (control != NULL) {
        *control = byte;
      }
      *state = CONTROL_RCV;
    }
    break;

  case CONTROL_RCV:
    if (control != NULL && byte == (currentA ^ (*control))) {
      if (dataBuffer != NULL && dataIndex != NULL) {
        *dataIndex = 0;
        isEscaped = FALSE;
        dataFieldError = FALSE;
        if (*control == C_SET || *control == C_DISC || *control == C_UA ||
            *control == C_RR_0 || *control == C_RR_1 || *control == C_REJ_0 ||
            *control == C_REJ_1) {
          *state = BCC_OK;
        } else {
          *state = DATA_RCV;
        }
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
      if (byte == FLAG) {
        if (isEscaped) {
          dataFieldError = TRUE;
          isEscaped = FALSE;
        }
        *state = STOP;
      } else if (byte == ESC && !isEscaped) {
        isEscaped = TRUE;
      } else if (isEscaped) {
        isEscaped = FALSE;
        if (byte != (FLAG ^ ESC_XOR) && byte != (ESC ^ ESC_XOR)) {
          dataFieldError = TRUE;
        }
        if (*dataIndex < MAX_PAYLOAD_SIZE + 1) {
          dataBuffer[(*dataIndex)++] = byte ^ ESC_XOR;
        } else {
          dataFieldError = TRUE;
          *state = START;
        }
      } else {
        if (*dataIndex < MAX_PAYLOAD_SIZE + 1) {
          dataBuffer[(*dataIndex)++] = byte;
        } else {
          // Trama excedeu o tamanho maximo por causa de ruido
          dataFieldError = TRUE;
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

static int writeAllBytes(const unsigned char *bytes, int nBytes) {
  int totalWritten = 0;
  while (totalWritten < nBytes) {
    int written =
        writeBytesSerialPort(bytes + totalWritten, nBytes - totalWritten);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
    if (written == 0) {
      return -1;
    }
    totalWritten += written;
  }
  return totalWritten;
}

// Função auxiliar de enviar a trama de 5 bytes, feita para reutilizar o código
static int sendTrama(unsigned char address, unsigned char control) {
  unsigned char frame[5];
  frame[0] = FLAG;
  frame[1] = address;
  frame[2] = control;
  frame[3] = address ^ control;
  frame[4] = FLAG;

  return writeAllBytes(frame, 5);
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
    closeSerialPort();
    return -1;
  }

  alarmCount = 0;
  alarmEnabled = FALSE;
  alarmTriggered = FALSE;

  States state = START;
  unsigned char receivedC = 0;

  while (alarmCount < llParameters.nRetransmissions && state != STOP) {
    if (alarmEnabled == FALSE) {
      int bytes = sendTrama(A_TX, C_SET);
      if (bytes < 0) {
        alarm(0);
        closeSerialPort();
        return -1;
      }
      printf("%d bytes written (SET frame sent, attempt #%d)\n", bytes,
             alarmCount + 1);
      alarmTriggered = FALSE;
      alarm(llParameters.timeout);
      alarmEnabled = TRUE;
      state = START; // incializa a state machine sempre que inicia o alarme
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        processAlarmTrigger();
        continue;
      }
      alarm(0);
      closeSerialPort();
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, &receivedC, NULL, NULL);
      if (state == STOP && receivedC != C_UA) {
        state = START;
      }
    }
    processAlarmTrigger();
  }

  alarm(0);
  alarmTriggered = FALSE;

  if (state == STOP) {
    printf("UA frame successfully received and verified via State Machine!\n");
    return 0;
  }

  fprintf(stderr, "Error: Connection timed out after %d attempts\n",
          llParameters.nRetransmissions);
  closeSerialPort();
  return -1;
}

int llOpenRx(LinkLayer llParameters) {

  parameters = llParameters;
  rxSequenceNumber = 0;
  dataFieldError = FALSE;
  rxDiscReceived = FALSE;
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
      closeSerialPort();
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
  if (bytes < 0) {
    closeSerialPort();
    return -1;
  }
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
  if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE) {
    fprintf(stderr, "Error: Invalid buffer or buffer size (%d) in llSend\n",
            bufSize);
    return -1;
  }

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
  alarmTriggered = FALSE;

  unsigned char expectedRR = (txSequenceNumber == 0) ? C_RR_1 : C_RR_0;
  unsigned char expectedREJ = (txSequenceNumber == 0) ? C_REJ_0 : C_REJ_1;

  States state = START;
  unsigned char receivedC = 0;

  while (alarmCount < parameters.nRetransmissions) {
    if (alarmEnabled == FALSE) {
      int bytes = writeAllBytes(trama, tramaSize);
      if (bytes < 0) {
        alarm(0);
        return -1;
      }
      statsFramesSent++;
      printf("%d bytes written (I frame sent, seq %d, attempt #%d)\n", bytes,
             txSequenceNumber, alarmCount + 1);
      alarmTriggered = FALSE;
      alarm(parameters.timeout);
      alarmEnabled = TRUE;
      state = START;
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        processAlarmTrigger();
        continue;
      }
      alarm(0);
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, &receivedC, NULL, NULL);

      if (state == STOP) {
        if (receivedC == expectedRR) {
          alarm(0);
          alarmTriggered = FALSE;
          printf("Positive ACK (RR) received!\n");
          txSequenceNumber ^= 1; // Alterna a sequência (0 -> 1 ou 1 -> 0)
          return tramaSize;
        } else if (receivedC == expectedREJ) {
          alarm(0);
          alarmTriggered = FALSE;
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
    processAlarmTrigger();
  }

  alarm(0);
  alarmTriggered = FALSE;
  fprintf(stderr, "Error: llSend timed out after %d attempts\n",
          parameters.nRetransmissions);
  return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////

int llReceive(unsigned char *packet) {
  if (packet == NULL) {
    return -1;
  }

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

  // Se o UA inicial se perdeu, o emissor retransmite SET: responder com UA
  if (receivedC == C_SET && index == 0 && statsFramesReceived == 0) {
    if (sendTrama(A_TX, C_UA) < 0) {
      return -1;
    }
    printf("Repeated SET frame received! Resent UA\n");
    return 0;
  }

  // Se o emissor abortou antecipadamente e enviou DISC, registar e interromper
  if (receivedC == C_DISC && index == 0) {
    rxDiscReceived = TRUE;
    printf("DISC frame received during llReceive (early close from TX)!\n");
    return -1;
  }

  // Validar que se trata efetivamente de uma trama de informacao (I)
  if (receivedC != C_I_0 && receivedC != C_I_1) {
    return 0;
  }

  unsigned char expectedC = (rxSequenceNumber == 0) ? C_I_0 : C_I_1;

  // Verificar primeiro se a trama e duplicada: segundo o guiao, tramas
  // duplicadas com cabecalho valido devem ser confirmadas com RR mesmo que
  // tenham erro no campo de dados ou BCC2
  if (receivedC != expectedC) {
    statsDuplicates++;
    unsigned char rr = (rxSequenceNumber == 0) ? C_RR_0 : C_RR_1;
    if (sendTrama(A_TX, rr) < 0) {
      return -1;
    }
    printf("Duplicate frame received! Resent RR\n");
    return 0;
  }

  // Se houve erro de stuffing (ex.: ESC + FLAG ou escape invalido) numa trama
  // nova, rejeitar imediatamente com REJ sem aceitar os dados
  if (dataFieldError) {
    statsRejSent++;
    unsigned char rej = (receivedC == C_I_0) ? C_REJ_0 : C_REJ_1;
    if (sendTrama(A_TX, rej) < 0) {
      return -1;
    }
    printf("Malformed data field (stuffing error) detected! REJ frame sent\n");
    return 0;
  }

  if (index < 2) {
    return 0; // trama vazia ou ruido: ignora e deixa continuar
  }

  // o ultimo byte recebido e o bcc2, o resto sao os dados
  int dataSize = index - 1;
  if (dataSize > MAX_PAYLOAD_SIZE) {
    return 0;
  }

  unsigned char receivedBcc2 = destuffed[index - 1];

  // calcular bcc2 dos dados recebidos para comparar
  unsigned char calculatedBcc2 = 0;
  for (int i = 0; i < dataSize; i++) {
    calculatedBcc2 ^= destuffed[i];
  }

  // se for trama nova e o bcc2 calculado der diferente, manda rej
  if (calculatedBcc2 != receivedBcc2) {
    statsRejSent++;
    unsigned char rej = (receivedC == C_I_0) ? C_REJ_0 : C_REJ_1;
    if (sendTrama(A_TX, rej) < 0) {
      return -1;
    }
    printf("BCC2 error detected! REJ frame sent\n");
    return 0;
  }

  // trama nova e correta: copia dados para o packet, alterna a sequencia e
  // manda rr
  for (int i = 0; i < dataSize; i++) {
    packet[i] = destuffed[i];
  }
  rxSequenceNumber ^= 1;
  statsFramesReceived++;
  unsigned char rr = (rxSequenceNumber == 0) ? C_RR_0 : C_RR_1;
  if (sendTrama(A_TX, rr) < 0) {
    return -1;
  }
  printf("Frame accepted (seq %d)! RR frame sent\n",
         (receivedC == C_I_0) ? 0 : 1);
  return dataSize;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx() {
  struct sigaction act = {0};
  act.sa_handler = &alarmHandler;
  if (sigaction(SIGALRM, &act, NULL) == -1) {
    perror("sigaction");
    closeSerialPort();
    return -1;
  }

  alarmCount = 0;
  alarmEnabled = FALSE;
  alarmTriggered = FALSE;

  States state = START;
  unsigned char receivedC = 0;

  // envia disc e espera por disc do rx com alarmes
  while (alarmCount < parameters.nRetransmissions && state != STOP) {
    if (alarmEnabled == FALSE) {
      int bytes = sendTrama(A_TX, C_DISC);
      if (bytes < 0) {
        alarm(0);
        closeSerialPort();
        return -1;
      }
      printf("%d bytes written (DISC frame sent, attempt #%d)\n", bytes,
             alarmCount + 1);
      alarmTriggered = FALSE;
      alarm(parameters.timeout);
      alarmEnabled = TRUE;
      state = START;
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        processAlarmTrigger();
        continue;
      }
      alarm(0);
      closeSerialPort();
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_RX, &receivedC, NULL, NULL);
      if (state == STOP && receivedC != C_DISC) {
        state = START;
      }
    }
    processAlarmTrigger();
  }

  alarm(0);
  alarmTriggered = FALSE;

  if (state == STOP) {
    printf("DISC frame successfully received from RX!\n");

    // responde com ua para fechar a ligacao de vez
    if (sendTrama(A_RX, C_UA) < 0) {
      closeSerialPort();
      return -1;
    }
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
  struct sigaction act = {0};
  act.sa_handler = &alarmHandler;
  if (sigaction(SIGALRM, &act, NULL) == -1) {
    perror("sigaction");
    closeSerialPort();
    return -1;
  }

  States state = START;
  unsigned char receivedC = 0;
  unsigned char dummyBuf[MAX_PAYLOAD_SIZE + 10];
  int dummyIndex = 0;
  unsigned char duplicateC = (rxSequenceNumber == 0) ? C_I_1 : C_I_0;

  alarmCount = 0;
  alarmEnabled = FALSE;
  alarmTriggered = FALSE;

  // Se o DISC ainda nao foi recebido durante llReceive, espera pelo DISC
  // enviado pelo emissor, continuando a confirmar duplicados da ultima trama I
  if (!rxDiscReceived) {
    while (alarmCount < parameters.nRetransmissions && state != STOP) {
      if (alarmEnabled == FALSE) {
        alarmTriggered = FALSE;
        alarm(parameters.timeout);
        alarmEnabled = TRUE;
      }

      unsigned char byte;
      int res = readByteSerialPort(&byte);

      if (res < 0) {
        if (errno == EINTR) {
          processAlarmTrigger();
          continue;
        }
        alarm(0);
        closeSerialPort();
        return -1;
      } else if (res == 1) {
        updateStateMachine(&state, byte, A_TX, &receivedC, dummyBuf,
                           &dummyIndex);
        if (state == STOP) {
          if (receivedC == C_DISC && dummyIndex == 0) {
            break;
          } else if (receivedC == duplicateC) {
            statsDuplicates++;
            unsigned char rr = (rxSequenceNumber == 0) ? C_RR_0 : C_RR_1;
            if (sendTrama(A_TX, rr) < 0) {
              alarm(0);
              closeSerialPort();
              return -1;
            }
            printf("Duplicate frame received during close! Resent RR\n");
            alarm(parameters.timeout);
            alarmCount = 0;
            alarmTriggered = FALSE;
            alarmEnabled = TRUE;
            state = START;
          } else {
            state = START;
          }
        }
      }
      processAlarmTrigger();
    }

    alarm(0);
    alarmTriggered = FALSE;

    if (state != STOP) {
      fprintf(stderr, "Error: Waiting for DISC timed out (RX)\n");
      closeSerialPort();
      return -1;
    }

    printf("DISC frame received from TX!\n");
  } else {
    rxDiscReceived = FALSE;
    printf("Processing DISC frame already received from TX!\n");
  }

  // envia disc com endereco A_RX e espera pela confirmacao final com ua
  // protegendo com temporizador e retransmissoes
  alarmCount = 0;
  alarmEnabled = FALSE;
  alarmTriggered = FALSE;
  state = START;
  receivedC = 0;

  while (alarmCount < parameters.nRetransmissions && state != STOP) {
    if (alarmEnabled == FALSE) {
      int bytes = sendTrama(A_RX, C_DISC);
      if (bytes < 0) {
        alarm(0);
        closeSerialPort();
        return -1;
      }
      printf("%d bytes written (DISC frame sent back to TX, attempt #%d)\n",
             bytes, alarmCount + 1);
      alarmTriggered = FALSE;
      alarm(parameters.timeout);
      alarmEnabled = TRUE;
      state = START;
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      if (errno == EINTR) {
        processAlarmTrigger();
        continue;
      }
      alarm(0);
      closeSerialPort();
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, 0, &receivedC, NULL, NULL);
      if (state == STOP) {
        if (receivedC == C_UA) {
          break;
        } else if (receivedC == C_DISC) {
          // O emissor nao recebeu o nosso DISC e reenviou o dele: responder ja
          if (sendTrama(A_RX, C_DISC) < 0) {
            alarm(0);
            closeSerialPort();
            return -1;
          }
          printf("Repeated DISC from TX! Resent DISC to TX\n");
          alarmTriggered = FALSE;
          alarm(parameters.timeout);
          alarmEnabled = TRUE;
          state = START;
        } else {
          state = START;
        }
      }
    }
    processAlarmTrigger();
  }

  alarm(0);
  alarmTriggered = FALSE;

  if (state != STOP) {
    fprintf(stderr, "Error: Waiting for final UA timed out (RX)\n");
    closeSerialPort();
    return -1;
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
