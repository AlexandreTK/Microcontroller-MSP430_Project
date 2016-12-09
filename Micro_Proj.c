/*
 *  MSP PINs
 * 
 *  I2C ACCEL - MPU6050
 *  P1.6          UCB0SCL
 *  P1.7          UCB0SDA
 *  
 *  BLUETOOTH MODULE - H05
 *  P1.1          TXD
 *  P1.2          RXD
 *  
 *  LED
 *  P1.0
 *
 */

#include <msp430g2553.h>
#include <legacymsp430.h>

/* 
 *  MPU6050 
 */
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H  0x41 
#define TEMP_OUT_L .0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define MPU_ADDRESS 0X68
#define PWR_MGMT_1   0x6B
#define STATUS_REGISTER 0x75

#define SCL_PIN BIT6
#define SDA_PIN BIT7
/* END */

/* 
 *  BLUETOOTH - UART
 */
#define RX BIT1
#define TX BIT2
/* END */

/* 
 *  OTHERS
 */
#define LED BIT0
#define DELAY_PUNCHES 100
#define PUNCH_THRESHOLD 70000 // 32 Bit
#define GAME_OVER_TIME 60000 // 10 Minutes, 100 = 1 Second
#define TIMER_PRECISION 10000 // 10ms, 1000 = 1ms
/* END */


/* 
 *  DATA STRUCTURES 
 */
struct MPU_Data {
  int xAccel;
  int yAccel;
  int zAccel;

  int xGyro;
  int yGyro;
  int zGyro;

  int temp;
  int statusRegister;

};
struct PlayerTime {
  unsigned char playerNum;
  unsigned int playerTime;
};
/* END */



/* 
 *  MPU6050 - I2C
 */
unsigned char RX_Data[15];
volatile unsigned char TX_Data[2];
volatile unsigned char RX_ByteCtr;
volatile unsigned char TX_ByteCtr;

int i2c_notready();
void i2cInit(void);
void i2cWrite(unsigned char);
void i2cRead(unsigned char);
//#pragma vector = USCIAB0TX_VECTOR
//__interrupt void USCIAB0TX_ISR(void)
/* END */


/* 
 *  BLUETOOTH - UART
 */
 volatile unsigned char data_in = '0';

 void setup_UART(void);
 //interrupt(USCIAB0RX_VECTOR) Receive_Data(void)
/* END */


/* 
 *  TIMER
 */
volatile unsigned int timer = 0;
volatile char timer_enabled = 0;

void Atraso_ms(volatile unsigned int x);
void Atraso_2(volatile unsigned int x);
void start_TIMER(void);
void stop_TIMER(void);
//interrupt(TIMER0_A1_VECTOR) TA0_ISR(void);
/* END */


/* 
 *  APP SPECIFIC
 */
struct PlayerTime bestPlayer;
 
void WakeUpMPU(void);
void SleepMPU(void);
int MPUStatus(void);
void getMPUData(struct MPU_Data * mpu_data);
long getAverageAcell(struct MPU_Data * mpu_data_ptr, volatile char startPos, unsigned char totalPos, unsigned char vecSize);
/* END */


/* 
 *  AUXILIARY FUNCTIONS
 */
int my_abs_16(int x);
long my_abs_32(long x);
unsigned int punchesPerSec(unsigned int numPunches, unsigned int timePunches);
void printMPUData(struct MPU_Data * mpu_data);
/* END */


/* 
 *  FUNCTIONS TO PRINT DATA
 */
void my_putc(char outData);
void my_puts(char *s);
static void xtoa(unsigned long x, const unsigned long *dp);
static void puth(unsigned n);
void my_printf(char *format, ...);
/* END */


int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;         // Stop WDT

  // Set clock speed (default = 1 MHz)
  BCSCTL1 = CALBC1_1MHZ;          // Basic Clock System CTL (1,8,12 16_MHZ available)
  DCOCTL  = CALDCO_1MHZ;          // Digitally-Controlled Oscillator CTL

  // set up I2C pins
  P1SEL |= SCL_PIN + SDA_PIN;         // Assign I2C pins to USCI_B0
  P1SEL2|= SCL_PIN + SDA_PIN;         // Assign I2C pins to USCI_B0


  Atraso_ms(10000);
  // Initialize the I2C state machine
  i2cInit();

  
  P1OUT &= ~LED;
  P1DIR |= LED;

  Atraso_ms(10000);
  setup_UART();


  struct MPU_Data mpu_data[10];
  volatile unsigned char mpu_vector_position = 0;
  volatile long next_punch = 0; // Used as delay from current punch to next punch to be measured

  unsigned char totalPlayers = 0;
  unsigned char currentPlayer = 0;
  unsigned char totalPunches = 200;
  volatile unsigned char punch = 0;

  unsigned char enabled = 0;
  
  while (1 | mpu_vector_position++)
  {
    
    // Save the last 10 data sets in the mpu_data[] array. 
    // Saves the data back to the first position of the array if it reaches the end (position 10).
    if(mpu_vector_position >= 10) {
      mpu_vector_position = 0;
    }

    // Start timer after the MPU has been hit once,
    if( punch == 1  && timer_enabled == 0 ) {
      start_TIMER();
    }

    // If "timer" variable reaches "GAME_OVER_TIME", prints "GAME OVER".
    if( timer == GAME_OVER_TIME ) {
      Atraso_2(50000); // Delay before sending data via bluetooth.
      my_printf("(4) GAME OVER");
      punch = 0;
      enabled = 0;
    }

    // If the player has punched the ball "totalPunches" times, check if he was the best player. If so, save his score.
    if( punch >= totalPunches) {
      stop_TIMER();
      if( timer < bestPlayer.playerTime || currentPlayer == 1 ) {
        bestPlayer.playerTime = timer;
        bestPlayer.playerNum = currentPlayer; 
      }
      punch = 0;
      enabled = 0;
      // If all the players have already played once, prints the winner and it's score (punches per second).
      if(currentPlayer >= totalPlayers) {
        Atraso_2(50000);
        my_printf("(4) Jogador%u-%uSPM", bestPlayer.playerNum, punchesPerSec(totalPunches -1, bestPlayer.playerTime));
      }
    }

    // Sets up the game. After receiving 'a', two inputs are expected:
    // - totalPunches : Number of punches each player will throw. (less than 255)
    // - totalPlayers: Number of players the game will have. (less than 255)
    if(data_in == 'a') {
      // Receiving the other data.
      data_in = 0; 
      int data_received = 0;
      while(data_received<2) {
        if(data_in != 0) {
          if(data_received==0) {
            totalPunches = data_in;
            data_received++;
          } else if(data_received==1) {
            totalPlayers = data_in;
            data_received++;
          }
          data_in = 0;
        }
      }
      // Setting up the game.
      punch = 0;      
      next_punch = 0;
      currentPlayer = 0;
      enabled = 0;
      my_printf("(1)%i, %i", totalPunches,totalPlayers);
      Atraso_2(50000);
      my_printf("(4) ");
        
      data_in = 0;
    }

    // Starts the game for the currentPlayer... Updates the currentPlayer and number of punches (zero).
    if(data_in == '1') {
      if (currentPlayer < totalPlayers) {
        punch = 0;
        next_punch = 0;
        currentPlayer++;
        my_printf("(2)%i  ", currentPlayer);
        my_printf("(3)%i  ", punch);
        enabled = 1;
        
        data_in = 0;  
      }        
    }

    // Check if the MPU is working... To do so reads the STATUS_REGISTER(0x75).
    if(data_in == '3') {
      if (MPUStatus() == 1){
           P1OUT ^= LED; // If it is working toggle the LED on and off.
      }
      data_in = 0;    
    }


    // Prints the last data sets received.
    if(data_in == '4') {
      printMPUData(&mpu_data[mpu_vector_position]);
      
      data_in = 0;    
    }  

    // Prints the last 10 MPU data sets received.
    if(data_in == '5') {
      for(volatile unsigned char i = 0; i< 10; i++) {
        printMPUData(&mpu_data[i]);
      }
         
      data_in = 0;    
    }  

    // Prints the average of the last data sets.. ( from 1 to 7 (or position -9 to -3) and 8 to 10 (or position -2 to 0) )
    if(data_in == '6') {
      
      // Values from -3(mpu_vector_position-3) to -9
      long avgBefore6 = getAverageAcell(mpu_data, mpu_vector_position - 3, 7, 10);
      my_printf("Accel - Avg Before%l\n",avgBefore6);

      // Values from 0(mpu_vector_position) to -2
      long avgAfter6 = getAverageAcell(mpu_data, mpu_vector_position, 3, 10);
      my_printf("Accel - Avg After%l\n",avgAfter6);

      data_in = 0;    
    }  

    // If received 's' as data, wake up the MPU.
    if(data_in == 's') {
      WakeUpMPU();
      
      data_in = 0;    
    }  

    // If received 'x' as data, disable the MPU.
    if(data_in == 'x') {
      SleepMPU();
      
      data_in = 0;    
    }  

    // Get the MPU Data
    getMPUData(&(mpu_data[mpu_vector_position]));
    Atraso_ms(10);


    // Wait "DELAY_PUNCHES" before analysing the last 10 data sets.
    if(next_punch <= DELAY_PUNCHES) {
      next_punch++;
    }
    // Check if the MPU has been hit
    else {
      if(enabled == 1) {
        long avgBefore = getAverageAcell(mpu_data, mpu_vector_position - 3, 7, 10);
        long avgAfter = getAverageAcell(mpu_data, mpu_vector_position, 3, 10);
        long dif = (avgAfter - avgBefore);
        if(my_abs_32(dif) > PUNCH_THRESHOLD && next_punch >= DELAY_PUNCHES) {
          punch++;
          my_printf("(3)%i ", punch);
          
          next_punch = 0;
        }
      }
    }

    
  }
  
}




/*
 * ****************************
 * APP SPECIFIC FUNCTIONS
 * ****************************
 * 
 * 
 */

// Enable the MPU
void WakeUpMPU(void)
{
  // Wake up the MPU-6050
  TX_Data[1] = 0x6B;            // address of PWR_MGMT_1 register
  TX_Data[0] = 0x00;            // set register to zero (wakes up the MPU-6050)
  TX_ByteCtr = 2;               // Two bytes to be written
  i2cWrite(MPU_ADDRESS);
}

// Disable the MPU
void SleepMPU(void)
{
  // Wake up the MPU-6050
  TX_Data[1] = 0x6B;            // address of PWR_MGMT_1 register
  TX_Data[0] = 0xFF;            // set register to zero (wakes up the MPU-6050)
  TX_ByteCtr = 2;               // Two bytes to be written
  i2cWrite(MPU_ADDRESS);
}

// Reads the "STATUS_REGISTER"(0x75), 
int MPUStatus(void)
{
    TX_Data[0] = STATUS_REGISTER;          // register address
    TX_ByteCtr = 1;                        // One byte to be written
    i2cWrite(MPU_ADDRESS);
    
    RX_ByteCtr = 1;                        // One byte to be read
    i2cRead(MPU_ADDRESS);
    if (RX_Data[0] == MPU_ADDRESS){
         return 1;
    }
    return 0;
}

// Saves the data received from the MPU to the MPU_Data structure.
void getMPUData(struct MPU_Data * mpu_data) 
{

    TX_Data[0] = ACCEL_XOUT_H;          // register address
    TX_ByteCtr = 1;
    i2cWrite(MPU_ADDRESS);

    // Read the two bytes of data and store them in zAccel
    RX_ByteCtr = 14;
    i2cRead(MPU_ADDRESS);
    mpu_data->xAccel  = RX_Data[13] << 8;        // MSB
    mpu_data->xAccel |= RX_Data[12];         // LSB
    mpu_data->yAccel  = RX_Data[11] << 8;        // MSB
    mpu_data->yAccel |= RX_Data[10];         // LSB
    mpu_data->zAccel  = RX_Data[9] << 8;        // MSB
    mpu_data->zAccel |= RX_Data[8];         // LSB

    mpu_data->xGyro  = RX_Data[5] << 8;        // MSB
    mpu_data->xGyro |= RX_Data[4];         // LSB
    mpu_data->yGyro  = RX_Data[3] << 8;        // MSB
    mpu_data->yGyro |= RX_Data[2];         // LSB
    mpu_data->zGyro  = RX_Data[1] << 8;        // MSB
    mpu_data->zGyro |= RX_Data[0];         // LSB
}

// Gets the average Accelaration on the X and Y axis. 
// Y axis weight is the double, and the Z axis is ignored.
long getAverageAcell(struct MPU_Data * mpu_data_ptr, volatile char startPos, unsigned char totalPos, unsigned char vecSize) 
{
        struct MPU_Data * mpu_data;
        
        volatile long sumX = 0;
        volatile long sumY = 0;
        volatile long sumZ = 0;
        
        for(volatile char i = startPos; i > (startPos - totalPos) ; i--) {
          if (i >= 0) {
            mpu_data = (mpu_data_ptr + i );
            sumX += mpu_data->xAccel;
            sumY += mpu_data->yAccel;
            sumZ += mpu_data->zAccel;
            //my_printf("%i) %i - AccelX = %i\n", i, i , mpu_data->xAccel);

          } else {
            mpu_data = (mpu_data_ptr +  (i + vecSize) );
            sumX += mpu_data->xAccel;
            sumY += mpu_data->yAccel;
            sumZ += mpu_data->zAccel;
            //my_printf("%i) %i - AccelX = %i\n", i, i + vecSize, mpu_data->xAccel);
          }

        }
        //return (sumX +sumY + sumZ)/totalPos;
        return (sumX +sumY*2)/totalPos;
}




/*
 * *********************************************************************************************
 * TIMER FUNCTIONS
 * *********************************************************************************************
 * 
 * 
 */
 
// Waits for x cycles
void Atraso_ms(volatile unsigned int x)
{
        volatile unsigned int aux1;
        aux1 = x;
        for(; aux1>0;aux1--) {
        }
}

// Waits for x^2 cycles
void Atraso_2(volatile unsigned int x)
{
        volatile unsigned int aux1;
        volatile unsigned int aux2;
        aux1 = x;
        aux2 = x;
        for(; aux1>0;aux1--) {
          for(; aux2>0;aux2--) {
          }
        }
}

// Starts the timer. 
void start_TIMER(void) 
{
  timer = 0;

  TACCR0 = TIMER_PRECISION - 1;
  TACTL = TASSEL_2 + ID_0 + MC_1 + TAIE + TACLR;
  timer_enabled = 1;
  
  _BIS_SR(GIE);
}

// Stops the timer. 
void stop_TIMER(void) 
{
  TACTL = MC_0;
  timer_enabled = 0;
}

// Timer A interruption. When it happens (every 10ms) increases the "timer" variable
interrupt(TIMER0_A1_VECTOR) TA0_ISR(void)
{
  timer++;
  //P1OUT ^= LED;
  TACTL &= ~TAIFG;
  
}




/*
 * *********************************************************************************************
 * I2C FUNCTIONS
 * *********************************************************************************************
 * 
 * 
 */

// if I2C is busy wait.
int i2c_notready()
{
        if(UCB0STAT & UCBBUSY) return 1;
        else return 0;
}

// Starts the I2C connection
void i2cInit(void)
{
  // set up I2C module
  UCB0CTL1 |= UCSWRST;        // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;         // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;      // Use SMCLK, keep SW reset
  UCB0BR0 = 10;         // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0CTL1 &= ~UCSWRST;       // Clear SW reset, resume operation
}

// Write values to I2C device
void i2cWrite(unsigned char address)
{
  while ( i2c_notready() );
  __disable_interrupt();
  UCB0I2CSA = address;        // Load slave address
  IE2 |= UCB0TXIE;        // Enable TX interrupt
  while(UCB0CTL1 & UCTXSTP);      // Ensure stop condition sent
  UCB0CTL1 |= UCTR + UCTXSTT;     // TX mode and START condition
  __bis_SR_register(CPUOFF + GIE);    // sleep until UCB0TXIFG is set ...
}

// Read values received from I2C
void i2cRead(unsigned char address)
{
  while ( i2c_notready() );
  __disable_interrupt();
  UCB0I2CSA = address;        // Load slave address
  IE2 |= UCB0RXIE;        // Enable RX interrupt
  while(UCB0CTL1 & UCTXSTP);      // Ensure stop condition sent
  UCB0CTL1 &= ~UCTR;        // RX mode
  UCB0CTL1 |= UCTXSTT;        // Start Condition
  __bis_SR_register(CPUOFF + GIE);    // sleep until UCB0RXIFG is set ...
}

// I2C interruption vector - Get/Sends data.
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  if (IFG2&UCB0RXIFG || IFG2&UCB0TXIFG) {
    if(UCB0CTL1 & UCTR)         // TX mode (UCTR == 1) && IFG2 & UCB0RXIFG
    {
      if (TX_ByteCtr)               // TRUE if more bytes remain
      {
        TX_ByteCtr--;       // Decrement TX byte counter
        UCB0TXBUF = TX_Data[TX_ByteCtr];  // Load TX buffer
      }
      else            // no more bytes to send
      {
        UCB0CTL1 |= UCTXSTP;      // I2C stop condition
        IFG2 &= ~UCB0TXIFG;     // Clear USCI_B0 TX int flag
        __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
      }
    }
    else  // (UCTR == 0)         // RX mode
    {
      RX_ByteCtr--;               // Decrement RX byte counter
      if (RX_ByteCtr)               // RxByteCtr != 0
      {
        RX_Data[RX_ByteCtr] = UCB0RXBUF;  // Get received byte
        if (RX_ByteCtr == 1)      // Only one byte left?
        UCB0CTL1 |= UCTXSTP;      // Generate I2C stop condition
      }
      else            // RxByteCtr == 0
      {
        RX_Data[RX_ByteCtr] = UCB0RXBUF;  // Get final received byte
        __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
      }
    }
  } else if (IFG2&UCA0RXIFG) {
    data_in = UCA0RXBUF;
  }
}




/*
 * *********************************************************************************************
 * UART FUNCTIONS
 * *********************************************************************************************
 * 
 * 
 */

// Sets up the UART connection.
void setup_UART(void)
{

        // Habilitar pinos para entrada e saída UART
        P1SEL2 = P1SEL |= RX+TX;

        // UART: desabilitar paridade, transmitir um byte
        // na ordem LSB->MSB, um bit de stop
        UCA0CTL0 = 0;
        // UART: utilizar SMCLK
        UCA0CTL1 = UCSSEL_2;
        // Baud rate de 9600 bps, utilizando oversampling
        UCA0BR0 = 6;
        UCA0BR1 = 0;
        UCA0MCTL = UCBRF_8 + UCOS16;
    
        // Habilitar interrupção por recebimento de byte via UART
        IE2 |= UCA0RXIE;
        
        // Habilitar interrupções
        _BIS_SR(GIE);
        // Entrar em modo de baixo consumo
        //_BIS_SR(LPM0_bits);
}

// Interrupção por recebimento de byte via UART
interrupt(USCIAB0RX_VECTOR) Receive_Data(void)
{

  if (IFG2&UCA0RXIFG) {
        // Guardar valor recebido via UART na variavel data_in
        data_in = UCA0RXBUF;

        //IFG2 &= ~UCA0RXIFG;
        LPM0_EXIT;
  }
}




/*
 * *********************************************************************************************
 * AUXILIARY FUNCTIONS
 * *********************************************************************************************
 * 
 * 
 */
 
// Calculates the absolute value of a integer of 16 bits.
int my_abs_16(int x)
{
  int y = (x >> 15);
  return (x ^ y) - y;
}

// Calculates the absolute value of a integer of 32 bits.
long my_abs_32(long x)
{
  long y = (x >> 31);
  return (x ^ y) - y;
}

// Calculates the amount of punches per second.
unsigned int punchesPerSec(unsigned int numPunches, unsigned int timePunches)
{
  //long minute = (1000000 / TIMER_PRECISION)*60;
  //return minute*numPunches/timePunches;
  
  unsigned long punSec = (unsigned long) 6000*((unsigned long)numPunches)/(unsigned long)timePunches;
  return ((unsigned int) punSec);
}

// Print the data receive from the MPU
void printMPUData(struct MPU_Data * mpu_data) {

  my_printf("\nAccel: \n");
  my_printf("|x = %i | y = %i | z = %i\n", mpu_data->xAccel, mpu_data->yAccel, mpu_data->zAccel );


  my_printf("Gyro: \n");
  my_printf("|x = %i | y = %i | z = %i\n", mpu_data->xGyro, mpu_data->yGyro, mpu_data->zGyro );
      
}




/*
 * *********************************************************************************************
 * FUNCTIONS TO PRINT DATA
 * *********************************************************************************************
 * 
 * 
 */

// Prints a char.
void my_putc(char outData)
{
    while((UCA0TXIFG & IFG2) == 0);
    UCA0TXBUF = outData;
}

// Prints a string.
void my_puts(char *s) 
{
    while(*s) my_putc(*s++); 
}

static const unsigned long dv[] = {
//  4294967296      // 32 bit unsigned max
    1000000000,     // +0
     100000000,     // +1
      10000000,     // +2
       1000000,     // +3
        100000,     // +4
//       65535      // 16 bit unsigned max     
         10000,     // +5
          1000,     // +6
           100,     // +7
            10,     // +8
             1,     // +9
};

// Prints data value.
static void xtoa(unsigned long x, const unsigned long *dp)
{
    char c;
    unsigned long d;
    if(x) {
        while(x < *dp) ++dp;
        do {
            d = *dp++;
            c = '0';
            while(x >= d) ++c, x -= d;
            my_putc(c);
        } while(!(d & 1));
    } else
        my_putc('0');
}

// Prints hegadecimal values
static void puth(unsigned n)
{
    static const char hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    my_putc(hex[n & 15]);
}
 // Prints formatted data
void my_printf(char *format, ...)
{
    char c;
    int i;
    long n;
    
    va_list a;
    va_start(a, format);
    while(c = *format++) {
        if(c == '%') {
            switch(c = *format++) {
                case 's':                       // String
                    my_puts(va_arg(a, char*));
                    break;
                case 'c':                       // Char
                    my_putc(va_arg(a, char));
                    break;
                case 'i':                       // 16 bit Integer
                case 'u':                       // 16 bit Unsigned
                    i = va_arg(a, int);
                    if(c == 'i' && i < 0) i = -i, my_putc('-');
                    xtoa((unsigned)i, dv + 5);
                    break;
                case 'l':                       // 32 bit Long
                case 'n':                       // 32 bit uNsigned loNg
                    n = va_arg(a, long);
                    if(c == 'l' &&  n < 0) n = -n, my_putc('-');
                    xtoa((unsigned long)n, dv);
                    break;
                case 'x':                       // 16 bit heXadecimal
                    i = va_arg(a, int);
                    puth(i >> 12);
                    puth(i >> 8);
                    puth(i >> 4);
                    puth(i);
                    break;
                case 0: return;
                default: goto bad_fmt;
            }
        } else
bad_fmt:    my_putc(c);
    }
    va_end(a);
}