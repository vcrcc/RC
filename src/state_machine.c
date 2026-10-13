#include "state_machine.h"
#include "serial_port.h"
#include <stdio.h>

#ifndef FALSE
#define FALSE 0
#endif

extern int alarmEnabled;
extern int alarmCount;

int receiveTrama(unsigned char expectedA, unsigned char expectedC) {
  State state = START;
  unsigned char byte = 0;

  while (state != STOP_STATE) {
    int bytes = readByteSerialPort(&byte);
    if (bytes > 0) {
      switch (state) {
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
          state = BCC_RCV;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case BCC_RCV:
        if (byte == FLAG)
          state = STOP_STATE;
        else
          state = START;
        break;
      default:
        state = START;
        break;
      }
    } else {
      if (alarmEnabled == FALSE && alarmCount > 0) {
        return -1;
      }
    }
  }

  return 0;
}

int receiveTramaRRREJ(unsigned char expectedA, unsigned char expectedRR, unsigned char expectedREJ) {
  State state = START;
  unsigned char byte = 0;
  int isREJ = 0;

  while (state != STOP_STATE) {
    int bytes = readByteSerialPort(&byte);
    if (bytes > 0) {
      switch (state) {
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
        if (byte == expectedRR) {
          state = C_RCV;
          isREJ = 0;
        } else if (byte == expectedREJ) {
          state = C_RCV;
          isREJ = 1;
        }
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case C_RCV:
        if (byte == (expectedA ^ (isREJ ? expectedREJ : expectedRR)))
          state = BCC_RCV;
        else if (byte == FLAG)
          state = FLAG_RCV;
        else
          state = START;
        break;
      case BCC_RCV:
        if (byte == FLAG)
          state = STOP_STATE;
        else
          state = START;
        break;
      default:
        state = START;
        break;
      }
    } else {
      if (alarmEnabled == FALSE && alarmCount > 0) {
        return -1;
      }
    }
  }

  return isREJ ? -2 : 1;
}
