/*
 * I2C_Accel_MPU6050
 *  P1.6          UCB0SCL
 *  P1.7          UCB0SDA
 *
 */
#include <msp430g2553.h>
#include <legacymsp430.h>

/* 
 *  MPU6050 
 */
#define PWR_MGMT_1   0x6B

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
 *  MPU6050 
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

unsigned char RX_Data[15];
unsigned char TX_Data[2];
unsigned char RX_ByteCtr;
unsigned char TX_ByteCtr;

struct MPU_Data mpu_data;


void i2cInit(void);
void i2cWrite(unsigned char);
void i2cRead(unsigned char);
void WakeUpMPU(void);
/* END */


/* 
 *  BLUETOOTH - UART
 */
 void setup_UART(void);

 volatile unsigned char data_in = '0';
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

  setup_UART();
  volatile unsigned char frase[] = "1) Test I2C\n";
  displayMsg(frase);

  // Initialize the I2C state machine
  i2cInit();
  WakeUpMPU();


      
  while (1)MS
  {

      
    if(data_in == '2') {
      volatile unsigned char frase3[] = "2) Test I2C\n";
      displayMsg(frase3);

      data_in = 0;          
    }
    
    if(data_in == '3') {
      if (MPUStatus() == 1){
           P1DIR |= BIT0;
           P1OUT ^= BIT0;
      }
      data_in = 0;    
    }


    getMPUData(&mpu_data);

    if(data_in == '4') {

    
    printMPUData(&mpu_data);
         
      data_in = 0;    
    }  

  }
}


void getMPUData(struct MPU_Data * mpu_data) {

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

void printMPUData(struct MPU_Data * mpu_data) {

    volatile unsigned char value[10];
    
    volatile unsigned char desc[] ="\nAccel (x - y - z):\n";
    displayMsg(desc);
    intToCharArray(value, mpu_data->xAccel, ' ');
    displayMsg(value);
    intToCharArray(value, mpu_data->yAccel, ' ');
    displayMsg(value); 
    intToCharArray(value, mpu_data->zAccel, '\n');
    displayMsg(value); 
    
    volatile unsigned char desc2[] ="Gyro (x - y - z):\n";
    displayMsg(desc2);
    intToCharArray(value, mpu_data->xGyro, ' ');
    displayMsg(value);
    intToCharArray(value, mpu_data->yGyro, ' ');
    displayMsg(value); 
    intToCharArray(value, mpu_data->zGyro, '\n');
    displayMsg(value); 
         
}


void intToCharArray(volatile unsigned char * c_arr, int number, char last)
{
  if(number < 0) {
    c_arr[0] = '-';
    c_arr[1] = (unsigned char)((-1*number/10000)) + '0';
    c_arr[2] = (unsigned char)((-1*number%10000)/1000) + '0';
    c_arr[3] = (unsigned char)((-1*number%1000)/100) + '0';
    c_arr[4] = (unsigned char)((-1*number%100)/10) + '0';
    c_arr[5] = (unsigned char)((-1*number%10)) + '0';
    c_arr[6] = last;
    c_arr[7] = '\0';
  } else {
    c_arr[0] = (unsigned char)((number/10000)) + '0';
    c_arr[1] = (unsigned char)((number%10000)/1000) + '0';
    c_arr[2] = (unsigned char)((number%1000)/100) + '0';
    c_arr[3] = (unsigned char)((number%100)/10) + '0';
    c_arr[4] = (unsigned char)((number%10)) + '0';
    c_arr[5] = last;
    c_arr[6] = '\0';
  }
}

void unsignedIntToCharArray(volatile unsigned char * c_arr, unsigned int number, char last)
{
  c_arr[0] = (unsigned char)((number/10000)) + '0';
  c_arr[1] = (unsigned char)((number%10000)/1000) + '0';
  c_arr[2] = (unsigned char)((number%1000)/100) + '0';
  c_arr[3] = (unsigned char)((number%100)/10) + '0';
  c_arr[4] = (unsigned char)((number%10)) + '0';
  c_arr[5] = last;
  c_arr[6] = '\0';
}

void displayMsg(volatile unsigned char frase[])
{
        volatile unsigned char count = 0;
        while(1)
        {
              
              while((UCA0TXIFG & IFG2) == 0);
              UCA0TXBUF = frase[count];
              if(frase[count]=='\0')
                      return;
              else
                      count++;
        }
}


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
        // Guardar valor recebido via UART na variavel data_in
        data_in = UCA0RXBUF;
        //sendChar(data_in);
        LPM0_EXIT;
}




void WakeUpMPU(void)
{
  // Wake up the MPU-6050
  TX_Data[1] = 0x6B;            // address of PWR_MGMT_1 register
  TX_Data[0] = 0x00;            // set register to zero (wakes up the MPU-6050)
  TX_ByteCtr = 2;
  i2cWrite(MPU_ADDRESS);
}

int MPUStatus(void)
{
    TX_Data[0] = STATUS_REGISTER;          // register address
    TX_ByteCtr = 1;
    i2cWrite(MPU_ADDRESS);
    
    RX_ByteCtr = 1;
    i2cRead(MPU_ADDRESS);
    if (RX_Data[0] == MPU_ADDRESS){
         return 1;
    }
    return 0;
}


//*********************************************************************************************
// if I2C is busy wait.
int i2c_notready(){
        if(UCB0STAT & UCBBUSY) return 1;
        else return 0;
}
//*********************************************************************************************
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

//*********************************************************************************************
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

//*********************************************************************************************
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

/**********************************************************************************************/
// USCIAB0TX_ISR
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

