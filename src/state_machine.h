#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#define FLAG 0x7E

typedef enum{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_RCV,
    STOP_STATE,
}State;

State updateSupervisionState(State currentState, unsigned char byte, unsigned char expectedA, unsigned char expectedC);
int receiveTrama(unsigned char expectedA, unsigned char expectedC);

#endif // STATE_MACHINE_H
