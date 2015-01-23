#include <msp430.h>
#include <driverlib.h>
#include <hw_regaccess.h>
#include <hw_memmap.h>
//#include <eusci_a_spi.h>

#include "usbConfig/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 // USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "usbApp/usbConstructs.h"

volatile uint8_t bHID_DataReceived_event = FALSE;  // Flags set by event handler
volatile uint8_t bCDC_DataReceived_event = FALSE;  // to indicate data has been
#define SPICLK 100000

uint8_t byteNo = 0; // TODO
uint8_t thermocoupleData[12];
uint8_t thermocoupleTransferPending = 0;

void initPorts (void)
{
        // Ustaw p1.0 jako wyjście.
        P1DIR |= GPIO_PIN0;
        // Prąd duży.
        P1DS |= GPIO_PIN0;
        // Stan high.
        P1OUT |= GPIO_PIN0;

        P4DIR |= GPIO_PIN7;
        P4OUT &= ~GPIO_PIN7;

        // Pin do sterowania przekaźnikiem.
        P2DIR |= GPIO_PIN5;
        P2OUT &= ~GPIO_PIN5;

        // P2.1 jako wejście
        P2DIR &= ~GPIO_PIN1;
        // Włącz rezystor pull.
        P2REN |= GPIO_PIN1;
        // Pull up, bo guzik zwiera do masy.
        P2OUT |= GPIO_PIN1;
        //Włącz przerwania
        P2IE |= GPIO_PIN1;
        // Zbocze opadające.
        P2IES |= GPIO_PIN1;

        // P1.1 jako wejście
        P1DIR &= ~GPIO_PIN1;
        // Włącz rezystor pull.
        P1REN |= GPIO_PIN1;
        // Pull up, bo guzik zwiera do masy.
        P1OUT |= GPIO_PIN1;
        //Włącz przerwania
        P1IE |= GPIO_PIN1;
        // Zbocze opadające.
        P1IES |= GPIO_PIN1;
}

/****************************************************************************/

void initClocks (void)
{
        GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P5, GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5);

        // Wyłączenie bypasu (domyślne). Bypass się włącza kiedy doprowadzamy "gotowy" sygnał zegarowy do
        // wejścia XIN.
        UCSCTL6 &= ~XT2BYPASS;

        // Drive stregth. Domyślnie jest ustawiony największy prąd oscylatora, co zapewnia mu najszybszy
        // start, ale też największe zużycie energii.
        UCSCTL6 &= ~(XT2DRIVE0 | XT2DRIVE1); // Wyczyść
        UCSCTL6 |= (XT2DRIVE0 | XT2DRIVE1); // Ustaw co tam chcesz.

        // Włączenie oscylatora XT2OSC. To się chyba powinno samo przestawić automatycznie i potem można go
        // po prostu wyłączyć.
        UCSCTL6 &= ~XT2OFF;

        uint16_t timeout = 50000;
        while ((UCSCTL7 & XT2OFFG) && --timeout) {
                //Clear OSC flaut Flags
                UCSCTL7 &= ~(XT2OFFG);

                //Clear OFIFG fault flag
                SFRIFG1 &= ~OFIFG;
        }

        // Wyłączenie bypassu i wyłączenie high frequency
        UCSCTL6 &= ~(XT1BYPASS | XTS);

        // Drive stregth.
        UCSCTL6 |= (XT1DRIVE0 | XT1DRIVE1);

        // Włączenie oscylatora XT2OSC. To się chyba powinno samo przestawić automatycznie i potem można go
        // po prostu wyłączyć.
        UCSCTL6 &= ~XT1OFF;

        timeout = 50000;
        while ((UCSCTL7 & XT1LFOFFG) && --timeout) {
                //Clear OSC flaut Flags
                UCSCTL7 &= ~(XT1LFOFFG);

                //Clear OFIFG fault flag
                SFRIFG1 &= ~OFIFG;
        }

        /*
         * FLL
         * Patrz rysunek po kolei:
         * Ustawienie SELREF, czyli źródla FLL (domyślnie XT1CLK)
         * FLLREFDIV__1    : Reference Divider: f(LFCLK)/1
         * FLLREFDIV__2    : Reference Divider: f(LFCLK)/2
         * FLLREFDIV__4    : Reference Divider: f(LFCLK)/4
         * FLLREFDIV__8    : Reference Divider: f(LFCLK)/8
         * FLLREFDIV__12   : Reference Divider: f(LFCLK)/12
         * FLLREFDIV__16   : Reference Divider: f(LFCLK)/16
         *                 :
         * SELREF__XT1CLK  : Multiply Selected Loop Freq. By XT1CLK (DEFAULT)
         * SELREF__REFOCLK : Multiply Selected Loop Freq. By REFOCLK
         * SELREF__XT2CLK  : Multiply Selected Loop Freq. By XT2CLK
         *
         * Znów operator "=", bo nie ma w tym rejestrze nic innego prócz tych 2 ustawień
         */
        UCSCTL3 = SELREF__XT2CLK | FLLREFDIV__4;

        /*
         * FLLN = 24 oraz FLLD = 1; Mnożnik 6 oznacza 1 * (24 + 1) = 25MHz
         *
         * FLLD__1  : Multiply Selected Loop Freq. By 1
         * FLLD__2  : Multiply Selected Loop Freq. By 2
         * FLLD__4  : Multiply Selected Loop Freq. By 4
         * FLLD__8  : Multiply Selected Loop Freq. By 8
         * FLLD__16 : Multiply Selected Loop Freq. By 16
         * FLLD__32 : Multiply Selected Loop Freq. By 32
         */
        UCSCTL2 = 24 | FLLD__1;

        /*
         * Przedział częstotliwości. Trzeba znaleźć w datasheet. Na przykład w MSP430F5506
         * tabela jest na 58 stronie i DCORSEL_5 odpowiada przedziałowi 2.5-6.0 MHz
         *
         * Dla MSP430F5529 tabelkę znajdziemy na 61 stronie. Każde DCORSELx występuje tam
         * podwójnie : 2 wiersze dla 0, 2 dla 1 i tak dalej. Nasza częstość dolna, to będzie MAX
         * z pierwszego wiersza, a górna, to MIN z drugiego wiersza. Dla tego dla 4MHz mamy
         * DCORSEL_3?.
         */
        UCSCTL1 = DCORSEL_6;

        // Oczekiwanie na ustabilizowanie się oscylatora.
        while (UCSCTL7_L & DCOFFG) {
            //Clear OSC flaut Flags
            UCSCTL7_L &= ~(DCOFFG);

            //Clear OFIFG fault flag
            SFRIFG1 &= ~OFIFG;
        }

        /**
         * Ustalamy źródło dla każdego z trzech sygnałów zegarowych:
         *
         * SELM__XT1CLK    : MCLK Source Select XT1CLK
         * SELM__VLOCLK    : MCLK Source Select VLOCLK
         * SELM__REFOCLK   : MCLK Source Select REFOCLK
         * SELM__DCOCLK    : MCLK Source Select DCOCLK
         * SELM__DCOCLKDIV : MCLK Source Select DCOCLKDIV
         * SELM__XT2CLK    : MCLK Source Select XT2CLK
         *
         * SELS__XT1CLK    : SMCLK Source Select XT1CLK
         * SELS__VLOCLK    : SMCLK Source Select VLOCLK
         * SELS__REFOCLK   : SMCLK Source Select REFOCLK
         * SELS__DCOCLK    : SMCLK Source Select DCOCLK
         * SELS__DCOCLKDIV : SMCLK Source Select DCOCLKDIV
         * SELS__XT2CLK    : SMCLK Source Select XT2CLK
         *                 :
         * SELA__XT1CLK    : ACLK Source Select XT1CLK
         * SELA__VLOCLK    : ACLK Source Select VLOCLK
         * SELA__REFOCLK   : ACLK Source Select REFOCLK
         * SELA__DCOCLK    : ACLK Source Select DCOCLK
         * SELA__DCOCLKDIV : ACLK Source Select DCOCLKDIV
         * SELA__XT2CLK    : ACLK Source Select XT2CLK
         *
         * Uwaga! Użyłem operatora =, a nie |=, żeby nadpisać wykluczające się ustawienia!
         */
        UCSCTL4 = SELM__DCOCLK | SELS__DCOCLK | SELA__REFOCLK;

        UCS_setExternalClockSource (32768, 4000000);
}

/****************************************************************************/

void initTimers (void)
{
        /*
         * Konfiguracja pierwszego timera typu A. Wybór źródła sygnału zegarowego za pomocą :
         * TASSEL__TACLK : TAxCLK sygnał zewnętrzny. ?
         * TASSEL__ACLK  : ACLK sygnał wewnętrzny. Auxiliary clock.
         * TASSEL__SMCLK : SMCLK sygnał wewnętrzny. Subsystem master clock.
         * TASSEL__INCLK : INCLK sygnał zewnętrzny. ?
         *
         */
        TA1CTL |= TASSEL__ACLK |

        /*
         * Pierwszy divider. Możliwe opcje to:
         * ID__1 : /1
         * ID__2 : /2
         * ID__4 : /4
         * ID__8 : /8
         */
        ID__1 |

        // Włącz przerwanie
        TAIE;

        /*
         * Dalszy podział sygnału zegarowego. Możliwe wartości:
         * TAIDEX_0 : /1
         * TAIDEX_1 : /2
         * TAIDEX_2 : /3
         * TAIDEX_3 : /4
         * TAIDEX_4 : /5
         * TAIDEX_5 : /6
         * TAIDEX_6 : /7
         * TAIDEX_7 : /8
         */
        TA1EX0 = TAIDEX_0;

        /*
         * Reset timera. Zawsze wykonać po ustawieniu dzielników sygnału zegarowego.
         */
        TA1CTL |= TACLR;

        TA1CCR0 = 0x7fff;
//        TA1CCR1 = 0x7fff;
//        TA1CCTL1 = OUTMOD_0 | CCIE;

        /*
         * Tryb działania
         * MC__STOP          : 0 - Stop
         *
         * MC__UP            : 1 - Up to TAxCCR0 i znów od 0. W tym przypadku używany
         *                         jest tylko TAxCCR0 (Timer_Ax Capture/Compare 0).
         *                         Kiedy timer dojdzie do wartości TAxCCR0, to ustawiany
         *                         jest bit CCIFG w rejestrze TAxCCTL0. Natomiast
         *                         zaraz potem, kiedy  tylko timer się wyzeruje µC
         *                         ustawia bit TAIFG w rejestrze TAxCTL (Timer_Ax Control).
         *                         Czyli te zdarzenia następują zaraz po sobie.
         *
         * MC__CONTINUOUS    : 2 - Continuous up, czyli liczy do 0xffff i znów od zera.
         *                         Kiedy dojdzie do 0xffff, to ustawia TAIFG w TAxCTL
         *                         (Timer_Ax Control), tak samo jak w poprzednim wypadku.
         *
         * MC__UPDOWN        : 3 - Up/Down, cyzli od 0 do TAxCCR0 i potem do 0 i w kółko.
         *
         */
        TA1CTL |= MC__UP;
}

/****************************************************************************/

void initSpi (void)
{
        /*
         * Ports:
         * P2.7 : Clock signal output – USCI_A0 SPI master mode
         * P3.2 : Slave transmit enable – USCI_A0 SPI mode
         * P3.3 : Slave in, master out – USCI_A0 SPI mode
         * P3.4 : Slave out, master in – USCI_A0 SPI mode : NOT USED
         */
        GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P2, GPIO_PIN7);
        GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P3, /*GPIO_PIN2 |*/ GPIO_PIN3 | GPIO_PIN4);

        P2DIR |= GPIO_PIN7; // Ustaw jako wyjście.
        P2DS |= GPIO_PIN7; // Prąd duży.

        P3DIR |= GPIO_PIN2 | GPIO_PIN3; // Ustaw jako wyjście.
        P3DS |= GPIO_PIN2 | GPIO_PIN3; // Prąd duży.
        P3OUT |= GPIO_PIN2; // disable slave

        uint8_t returnValue = USCI_A_SPI_masterInit (USCI_A0_BASE,
                                                     USCI_A_SPI_CLOCKSOURCE_SMCLK,
                                                     UCS_getSMCLK (),
                                                     SPICLK,
                                                     USCI_A_SPI_MSB_FIRST,
                                                     USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
                                                     USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH);

        if (STATUS_FAIL == returnValue) {
//                cdcSendDataInBackground((uint8_t*) wholeString, strlen (wholeString), CDC0_INTFNUM, 1);
                return;
        }

//        USCI_A_SPI_select4PinFunctionality (USCI_A0_BASE, EUSCI_A_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE);

        //Enable SPI module
        USCI_A_SPI_enable (USCI_A0_BASE);

        //Enable Receive interrupt
        USCI_A_SPI_clearInterruptFlag (USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);
        USCI_A_SPI_enableInterrupt (USCI_A0_BASE, USCI_A_SPI_RECEIVE_INTERRUPT);
}

/****************************************************************************/

__attribute__((interrupt (PORT1_VECTOR)))
void port1 (void)
{
        P1OUT |= GPIO_PIN0;
//        P1IFG &= ~GPIO_PIN1;
//        volatile uint16_t y = P2IV;
        P1IV = 0;
}

__attribute__((interrupt (PORT2_VECTOR)))
void port2 (void)
{
        P1OUT &= ~GPIO_PIN0;
//        P2IFG &= ~GPIO_PIN1;
//        volatile uint16_t y = P2IV;
        P2IV = 0;
}

static volatile int III = 0;

__attribute__((interrupt(TIMER1_A1_VECTOR)))
void timeIterrupt (void)
{
        //uint8_t* wholeString = "hello!\r\n";
        //cdcSendDataInBackground((uint8_t*) wholeString, strlen (wholeString), CDC0_INTFNUM, 1);

        thermocoupleTransferPending = 1;
        P3OUT &= ~GPIO_PIN2; // enable slave
        __delay_cycles (100);

        //USCI_A0 TX buffer ready?
        while (!USCI_A_SPI_getInterruptStatus (USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT))
                ;

        //Transmit Data to slave
        USCI_A_SPI_transmitData (USCI_A0_BASE, 0x00);
//        P3OUT |= GPIO_PIN2; // disable slave





//        if (P4OUT & GPIO_PIN7) {
//                P2OUT |= GPIO_PIN5;
//        }
//        else {
//                P2OUT &= ~GPIO_PIN5;
//        }

        TA1CTL &= ~TAIFG;
        TA1CCTL1 &= ~CCIFG;
}

int32_t accumulatedError = 0;

__attribute__((interrupt(USCI_A0_VECTOR)))
void USCI_A0_ISR(void)
{
        __even_in_range(UCA0IV, 4);
//        switch (__even_in_range(UCA0IV, 4)) {
//        //Vector 2 - RXIFG
//        case 2:
                //USCI_A0 TX buffer ready?
                while (!USCI_A_SPI_getInterruptStatus (USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT))
                        ;

                thermocoupleData[byteNo++] = USCI_A_SPI_receiveData (USCI_A0_BASE);

                if (byteNo > 3) {
                        byteNo = 0;
                        P3OUT |= GPIO_PIN2; // disable slave
//                        cdcSendDataInBackground (thermocoupleData, 4, CDC0_INTFNUM, 1);
                        uint16_t temp = (((thermocoupleData[0] << 8) | thermocoupleData[1]) >> 2);
                        uint16_t sp = 100; // sp * 4;
                        int16_t e = (sp * 4) - temp; // Max od -1200 do 1200.
                        accumulatedError += e;

                        // TODO przekazywać w requeście.
                        uint16_t kp = 5;
                        uint16_t ki = 1;

                        int32_t p = (int32_t)kp * e;
                        int32_t i = ki * accumulatedError / 1000;
                        int32_t u =  p + i;

//                        thermocoupleData[4] = p >> 8;
//                        thermocoupleData[5] = p & 0xff;
//                        thermocoupleData[6] = i >> 8;
//                        thermocoupleData[7] = i & 0xff;

                        uint8_t *pp = (uint8_t *)&p;
                        thermocoupleData[4] = pp[0];
                        thermocoupleData[5] = pp[1];
                        thermocoupleData[6] = pp[2];
                        thermocoupleData[7] = pp[3];

                        pp = (uint8_t *)&i;
                        thermocoupleData[8] = pp[0];
                        thermocoupleData[9] = pp[1];
                        thermocoupleData[10] = pp[2];
                        thermocoupleData[11] = pp[3];

                        if (u < 0) {
                                P2OUT &= ~GPIO_PIN5;
                        }
                        else {
                                P2OUT |= GPIO_PIN5;
                        }

                        P4OUT ^= GPIO_PIN7;
                        thermocoupleTransferPending = 0;
                }
                else {
                        USCI_A_SPI_transmitData (USCI_A0_BASE, 0x00);
                }

                //Increment data
//                transmitData++;

                //Send next value
//                USCI_A_SPI_transmitData(USCI_A0_BASE, transmitData);

                //Delay between transmissions for slave to process information
//                __delay_cycles(40);

//                break;
//        default: break;
//        }
}

/****************************************************************************/

uint8_t getTempRequest (void)
{
//        usbClearOEP0ByteCount ();
        wBytesRemainingOnIEP0 = 12; //amount of data to be send over EP0 to host

        while (thermocoupleTransferPending)
                ;

        usbSendDataPacketOnEP0 ((uint8_t *) thermocoupleData); //send data to host
        P1OUT ^= GPIO_PIN0;
        return FALSE;
}

/****************************************************************************/

int main (void)
{
        WDTCTL = WDTPW | WDTHOLD;

        PMM_setVCore (PMM_CORE_LEVEL_3);

        initPorts ();
        initClocks ();
        initTimers ();
        initSpi ();
        USB_setup (TRUE, TRUE);  // Init USB & events; if a host is present, connect

        __enable_interrupt ();

        while (1) {
                uint8_t ReceiveError = 0, SendError = 0;
//                uint16_t count;

                // Check the USB state and loop accordingly
                switch (USB_connectionState()) {

                case ST_ENUM_ACTIVE:
                        __bis_SR_register(LPM0_bits + GIE);         // Enter LPM0 until awakened by an event handler
                        __no_operation();
                        P1OUT ^= GPIO_PIN0;
                        // For debugger

                        // Exit LPM because of a data-receive event, and fetch the received data

//                        if (bHID_DataReceived_event) {               // Message is received from HID application
//                                bHID_DataReceived_event = FALSE;        // Clear flag early -- just in case execution breaks below because of an error
//                                count = hidReceiveDataInBuffer((uint8_t*) dataBuffer, BUFFER_SIZE, HID0_INTFNUM);
//
//                                uint16_t load16 = 0;
//
//                                if (count >= 2) {
//                                        load16 = (dataBuffer[0] << 8) | dataBuffer[1];
//                                }
//
//                                moveAbsolute (load16);
//
//                        }

//                        if (bCDC_DataReceived_event) { // Message is received from CDC application
//                                bCDC_DataReceived_event = FALSE; // Clear flag early -- just in case execution breaks below because of an error
//                                cdcReceiveDataInBuffer((uint8_t*) wholeString, MAX_STR_LENGTH, CDC0_INTFNUM);
//                                strncat(wholeString, ".\r\n ", 3);
//
//                                if (cdcSendDataInBackground((uint8_t*) wholeString, strlen(wholeString), CDC0_INTFNUM, 1)) {  // Send message to other CDC App
//                                        SendError = 0x01;
//                                        break;
//                                }
//
//                                memset(wholeString, 0, MAX_STR_LENGTH);   // Clear wholeString
//                        }

                        break;

                        // These cases are executed while your device is disconnected from
                        // the host (meaning, not enumerated); enumerated but suspended
                        // by the host, or connected to a powered hub without a USB host
                        // present.

                case ST_PHYS_DISCONNECTED:
                case ST_ENUM_SUSPENDED:
                case ST_PHYS_CONNECTED_NOENUM_SUSP:
                        __bis_SR_register(LPM3_bits + GIE);
                        _NOP();
                        break;

                        // The default is executed for the momentary state
                        // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
                        // seconds.  Be sure not to enter LPM3 in this state; USB
                        // communication is taking place here, and therefore the mode must
                        // be LPM0 or active-CPU.
                case ST_ENUM_IN_PROGRESS:
                default:
                        ;
                }

                if (ReceiveError || SendError) {
                        //TO DO: User can place code here to handle error
                }
        }

        return 0;
}



