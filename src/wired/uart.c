/**
 * @file uart.c
 * @author Aaron Parks -- Framework + TX logic
 * @author Ivar in 't Veen -- RX logic
 * @brief UART module for transmitting/receiving data using the USCI_A0 peripheral
 */

#include <msp430.h>
#include "uart.h"
#include "globals.h"

/**
 * State variables for the UART module
 */
typedef struct {
    uint8_t isTxBusy; // Is the module currently in the middle of a transmit operation?
    uint8_t* txPtr; // Pointer to the next byte to be transmitted
    uint16_t txBytesRemaining; // Number of bytes left to send

    uint8_t isRxBusy; // Is the module currently in the middle of a receive operation?
    uint8_t* rxPtr; // Pointer to the next byte to be received
    uint16_t rxBytesRemaining; // Maximum number of bytes left to receive
} uart_sm_t;

static volatile uart_sm_t UART_SM;

/**
 * Configure the eUSCI_A0 module in UART mode and prepare for UART transmission.
 *
 * @todo Currently assumes an 8MHz SMCLK. Make robust to clock frequency changes by using 32k ACLK.
 */
void UART_init(void) {

    // Configure USCI_A0 for UART mode
    UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK

    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 21-4: UCBRSx = 0x04
    // UCBRFx = int ( (52.083-52)*16) = 1

//#define UART_BAUDRATE 9600
#define UART_BAUDRATE 115200

//#define UART_CLOCK 1000000
//#define UART_CLOCK 4000000
#define UART_CLOCK 8000000

#if UART_CLOCK == 1000000
#if UART_BAUDRATE == 9600
    UCA0BR0 = 52;                             // 8000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW = UCOS16 | UCBRF_1;
#endif // UART_BAUDRATE
#elif UART_CLOCK == 4000000
#if UART_BAUDRATE == 9600
    UCA0BR0 = 26;                             // 8000000/16/9600
    UCA0BR1 = 0;
    UCA0MCTLW = UCOS16 | (0xB6 << 8);
#elif UART_BAUDRATE == 115200
    UCA0BR0 = 2;
    UCA0BR1 = 0;
    UCA0MCTLW = UCOS16 | UCBRF_2 | (0xBB << 8);
#endif // UART_BAUDRATE
#elif UART_CLOCK == 8000000
#if UART_BAUDRATE == 115200
    UCA0BR0 = 4;
    UCA0BR1 = 0;
    UCA0MCTLW = UCOS16 | UCBRF_4 | (0x55 << 8);
#endif // UART_BAUDRATE
#endif // UART_CLOCK

    PUART_TXSEL0 &= ~PIN_UART_TX; // TX pin to UART module
    PUART_TXSEL1 |= PIN_UART_TX;

    PUART_RXSEL0 &= ~PIN_UART_RX; // RX pin to UART module
    PUART_RXSEL1 |= PIN_UART_RX;

    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI

    // Initialize module state
    UART_SM.isTxBusy = FALSE;
    UART_SM.txBytesRemaining = 0;
    UART_SM.isRxBusy = FALSE;
    UART_SM.rxBytesRemaining = 0;

}

void UART_teardown()
{
    // disable UART
    // Not sure how to do this best, but set all UCA0* registers to
    // their default values.  See User's Guide for default values.
    PUART_TXSEL0 &= ~PIN_UART_TX;
    PUART_TXSEL1 &= ~PIN_UART_TX;
    PUART_RXSEL0 &= ~PIN_UART_RX;
    PUART_RXSEL1 &= ~PIN_UART_RX;
    UCA0CTLW0 = 0x0001;
    UCA0BR0 = 0x0000;
    UCA0MCTLW = 0x0000;
    UCA0IE = 0x0000;
    UCA0IFG = 0x0000;
}


/**
 * Transmit the contents of the given character buffer. Do not block.
 *
 * @param txBuf the character buffer to be transmitted
 * @param size the number of bytes to send
 */
void UART_asyncSend(uint8_t* txBuf, uint16_t size) {

    // Block until prior transmission has completed
    while (UART_SM.isTxBusy)
        ;

    // Set up for start of transmission
    UART_SM.isTxBusy = TRUE;
    UART_SM.txPtr = txBuf;
    UART_SM.txBytesRemaining = size;

    UCA0IFG &= ~(USCI_UART_UCTXIFG); // Clear the 'ready to accept byte' flag

    UCA0IE |= UCTXIE; // Enable USCI_A0 TX interrupt ('ready to accept byte')
    //UCA0TXBUF = *(UART_SM.txPtr++); // Load in first byte

    // The bytes are transmitted in the TX ISR (which is called whenever the
    // UART is ready to accept a byte), and the isBusy flag is cleared when the
    // last byte has *finished* transmitting.
}

/**
 * Transmit the contents of the given character buffer. Block until complete.
 *
 * @param txBuf the character buffer to be transmitted
 * @param size the number of bytes to send
 *
 */
void UART_send(uint8_t* txBuf, uint16_t size) {

    UART_asyncSend(txBuf, size);

    // Block until complete
    while (UART_SM.isTxBusy)
        ;
}

/**
 * Transmit the contents of the given character buffer. Block until complete,
 *  and use UART status register polling instead of interrupts.
 */
void UART_critSend(uint8_t* txBuf, uint16_t size) {

    // Block until prior transmission has completed
    while (UART_SM.isTxBusy)
        ;

    // Set up for start of transmission
    UART_SM.isTxBusy = TRUE;
    UART_SM.txPtr = txBuf;
    UART_SM.txBytesRemaining = size;

    UCA0IV &= ~(USCI_UART_UCTXIFG); // Clear byte completion flag

    while (UART_SM.txBytesRemaining--) {
        UCA0TXBUF = *(UART_SM.txPtr++); // Load in next byte
        while (!(UCA0IFG & UCTXIFG))
            ; // Wait for byte transmission to complete
        UCA0IFG &= ~(UCTXIFG); // Clear byte completion flag
    }

    UART_SM.isTxBusy = FALSE;
}

/**
 * Return true if UART TX module is in the middle of an operation, false if not.
 */
uint8_t UART_isTxBusy() {
    return UART_SM.isTxBusy;
}

/**
 * Receive character buffer. Do not block.
 *
 * @param rxBuf the character buffer to be received to
 * @param size the number of bytes to receive
 */
void UART_asyncReceive(uint8_t* rxBuf, uint16_t size) {

    // Block until prior reception has completed
    while (UART_SM.isRxBusy)
        ;

    // Set up for start of reception
    UART_SM.isRxBusy = TRUE;
    UART_SM.rxPtr = rxBuf;
    UART_SM.rxBytesRemaining = size;

    UCA0IFG &= ~(UCRXIFG); // Clear byte completion flag

    UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupt

    // The rest of the reception will be completed by the RX ISR (which
    //  will wake after each byte has been received), and the isBusy flag
    //  will be cleared when done.
}

/**
 * Receive character buffer. Block until complete.
 *
 * @param rxBuf the character buffer to be received to
 * @param size the number of bytes to receive
 *
 */
void UART_receive(uint8_t* rxBuf, uint16_t size) {

    UART_asyncReceive(rxBuf, size);

    // Block until complete
    while (UART_SM.isRxBusy)
        ;
}

/**
 * Receive to the given character buffer. Block until complete,
 *  and use UART status register polling instead of interrupts.
 */
void UART_critReceive(uint8_t* rxBuf, uint16_t size) {

    // Block until prior reception has completed
    while (UART_SM.isRxBusy)
        ;

    // Set up for start of reception
    UART_SM.isRxBusy = TRUE;
    UART_SM.rxPtr = rxBuf;
    UART_SM.rxBytesRemaining = size;

    UCA0IFG &= ~(UCRXIFG); // Clear byte completion flag

    while (UART_SM.rxBytesRemaining--) {
        while (!(UCA0IFG & UCRXIFG))
            ; // Wait for byte reception to complete
        UCA0IFG &= ~(UCRXIFG); // Clear byte completion flag

        uint8_t rec = UCA0RXBUF; // Read next byte
        *(UART_SM.rxPtr++) = rec; // Store byte
    }

    UART_SM.isRxBusy = FALSE;
}

/**
 * Return true if UART RX module is in the middle of an operation, false if not.
 */
uint8_t UART_isRxBusy() {
    return UART_SM.isRxBusy;
}

/**
 * Return true if UART RX module is not in the middle of an operation (e.g. done), false if not.
 *
 * Could be used in combination with UART_asyncReceive.
 */
uint8_t UART_isRxDone() {
    return !(UART_SM.isRxBusy);
}

/**
 * Handles transmit and receive interrupts for the UART module.
 * Interrupts typically occur once after each byte transmitted/received.
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t rec;

    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        if (UART_SM.rxBytesRemaining--) {
            rec = UCA0RXBUF; // Read next byte
            *(UART_SM.rxPtr++) = rec; // Store byte
        }

        if (0 == UART_SM.rxBytesRemaining) {
            UCA0IE &= ~(UCRXIE); // Disable USCI_A0 RX interrupt
            UART_SM.isRxBusy = FALSE;
        }

        break;
    case USCI_UART_UCTXIFG:
        UCA0TXBUF = *(UART_SM.txPtr++); // if interrupt was enabled, there must be bytes
        if (--UART_SM.txBytesRemaining == 0) {
            // TODO: actually, this wait should probably happen for blocking version only
            while (UCA0STATW & UCBUSY); // wait for last byte to finish transmitting
            UCA0IE &= ~(UCTXIE); // Disable USCI_A0 TX interrupt
            UART_SM.isTxBusy = FALSE;
        }
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        break;
    }
}

