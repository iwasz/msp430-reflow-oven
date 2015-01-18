#include <msp430.h>
#include <driverlib.h>
#include <hw_regaccess.h>
#include <hw_memmap.h>

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
        if (++III % 2 == 0) {
                P4OUT &= ~GPIO_PIN7;
        }
        else {
                P4OUT |= GPIO_PIN7;
        }

        TA1CTL &= ~TAIFG;
        TA1CCTL1 &= ~CCIFG;
}

/****************************************************************************/

int main (void)
{
        WDTCTL = WDTPW | WDTHOLD;

        PMM_setVCore (PMM_CORE_LEVEL_3);

        initPorts ();
        initClocks ();
        initTimers ();

        __enable_interrupt ();

        while (1) {
                P1OUT = 0x01;

                for (volatile uint16_t i = 0; i < 65535; ++i)
                                asm ("nop");

                P1OUT = 0x00;

                for (volatile uint16_t i = 0; i < 65535; ++i)
                                asm ("nop");
        }

        return 0;
}



