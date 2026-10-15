// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

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

static int alarmEnabled = FALSE;
static int alarmCount = 0;

// Alarm function handler.
// This function will run whenever the signal SIGALRM is received.
static void alarmHandler(int signal) {
  alarmEnabled = FALSE;
  alarmCount++;

  printf("Alarm #%d received\n", alarmCount);
}

typedef enum { START, FLAG_RCV, ADRESS_RCV, CONTROL_RCV, BCC_OK, STOP } States;

static void updateStateMachine(States *state, unsigned char byte, unsigned char expectedA, unsigned char expectedC) {
  switch (*state) {
  case START:
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
    if (byte == expectedC) {
      *state = CONTROL_RCV;
    } else if (byte == FLAG) {
      *state = FLAG_RCV;
    } else {
      *state = START;
    }
    break;

  case CONTROL_RCV:
    if (byte == (expectedA ^ expectedC)) {
      *state = BCC_OK;
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

  while (alarmCount < llParameters.nRetransmissions && state != STOP) {
    if (alarmEnabled == FALSE) {
      int bytes = sendTrama(A_TX, C_SET);
      printf("%d bytes written (SET frame sent, attempt #%d)\n", bytes,
             alarmCount + 1);
      alarm(llParameters.timeout);
      alarmEnabled = TRUE;
      state = START; //incializa a state machine sempre que inicia o alarme
    }

    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, C_UA);
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
  if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) {
    perror("openSerialPort");
    return -1;
  }

  printf("Serial port %s opened\n", llParameters.serialPort);

  States state = START;

  while (state != STOP) {
    unsigned char byte;
    int res = readByteSerialPort(&byte);

    if (res < 0) {
      return -1;
    } else if (res == 1) {
      updateStateMachine(&state, byte, A_TX, C_SET);
    }
  }

  printf("SET frame successfully received and verified via State Machine!\n");

  int bytes = sendTrama(A_TX, C_UA);
  printf("%d bytes written (UA frame sent)\n", bytes);

  return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize) {
  // TODO: Implement this function

  return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet) {
  // TODO: Implement this function

  return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx() {
  // TODO: Implement this function

  return 0;
}

int llCloseRx() {
  // TODO: Implement this function

  return 0;
}
