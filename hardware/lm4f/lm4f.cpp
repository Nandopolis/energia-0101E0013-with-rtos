

#include "Energia.h"
#include "libraries/SPI/SPI.h"

#include <stdarg.h>


void p(char *fmt, ... ){
        char buf[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 128, fmt, args);
        va_end (args);
        Serial.print(buf);
}



#define CSPIN 18
#define SOPIN 13

void setup()
{
  pinMode(RED_LED,OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  pinMode(BLUE_LED,OUTPUT);
  
  pinMode(CSPIN,OUTPUT);
  
  analogWrite(RED_LED,1);
  analogWrite(GREEN_LED,1);
  analogWrite(BLUE_LED,1);
  
  
  Serial.begin(115200);
  SPI.begin();
  
  //SPI.setModule(0);
  //SPI.setClockDivider(SPI_CLOCK_DIV128);
  //SPI.setDataMode(SPI_MODE0);
}









#define CC1101_CMD_SRES    0x30
#define CC1101_CMD_SFSTXON 0x31
#define CC1101_CMD_SXOFF   0x32
#define CC1101_CMD_SCAL    0x33
#define CC1101_CMD_SRX     0x34
#define CC1101_CMD_STX     0x35
#define CC1101_CMD_SILDE   0x36
#define CC1101_CMD_WOR     0x37 //undocumented
#define CC1101_CMD_SWOR    0x38
#define CC1101_CMD_SPWD    0x39
#define CC1101_CMD_SFRX    0x3A
#define CC1101_CMD_SFTX    0x3B
#define CC1101_CMD_SWORRST 0x3C
#define CC1101_CMD_SNOP    0x3D


typedef struct {
  uint8_t readyn: 1;
  uint8_t state: 3;
  uint8_t fifo: 4;
} cb_t  ; //__attribute__ ((__packed__));

char* cc1101_cb_str(uint8_t cb){

  cb_t* x=(cb_t*) &cb;
  
  //p("readyn=%d state=%d fifo=%d %d\r\n",x->readyn,x->state,x->fifo,sizeof(cb_t));
  
  switch( x->state ){
    case 0:  return "CB_IDLE            ";
    case 1:  return "CB_RX              ";
    case 2:  return "CB_TX              ";
    case 3:  return "CB_FSTXON          ";
    case 4:  return "CB_CALIBRATE       ";
    case 5:  return "CB_SETTLING        ";
    case 6:  return "CB_RXFIFO_OVERFLOW ";
    case 7:  return "CB_TXFIFO_UNDERFLOW";
    default: return "CB_UNKNOWN         ";
  
  }

}


char* cc1101_cmd_str(uint8_t cmd){
   switch(cmd){
      case CC1101_CMD_SRES:    return "CC1101_CMD_SRES   ";
      case CC1101_CMD_SFSTXON: return "CC1101_CMD_SFSTXON";
      case CC1101_CMD_SXOFF:   return "CC1101_CMD_SXOFF  ";
      case CC1101_CMD_SCAL:    return "CC1101_CMD_SCAL   ";
      case CC1101_CMD_SRX:     return "CC1101_CMD_SRX    ";
      case CC1101_CMD_STX:     return "CC1101_CMD_STX    ";
      case CC1101_CMD_SILDE:   return "CC1101_CMD_SILDE  ";
      case CC1101_CMD_WOR:     return "CC1101_CMD_WOR    ";
      case CC1101_CMD_SWOR:    return "CC1101_CMD_SWOR   ";
      case CC1101_CMD_SPWD:    return "CC1101_CMD_SPWD   ";
      case CC1101_CMD_SFRX:    return "CC1101_CMD_SFRX   ";
      case CC1101_CMD_SFTX:    return "CC1101_CMD_SFTX   ";
      case CC1101_CMD_SWORRST: return "CC1101_CMD_SWORRST";
      case CC1101_CMD_SNOP:    return "CC1101_CMD_SNOP   ";
      default:                 return "CC1101_CMD_UNKNOWN";
   }
}

uint8_t cc1101_cmd(uint8_t reg){

   SPI.begin();
   SPI.setModule(0);
   SPI.setDataMode(SPI_MODE0);
   
   #if (F_CPU == 1000000)
   SPI.setClockDivider(SPI_CLOCK_DIV1); //1 MHz SPI Clock  
   #elif (F_CPU == 16000000)
   SPI.setClockDivider(SPI_CLOCK_DIV2); //8 MHz SPI Clock
   #elif (F_CPU == 24000000)   
   SPI.setClockDivider(SPI_CLOCK_DIV4); //6 MHz SPI Clock
   #elif (F_CPU == 80000000)
   SPI.setClockDivider(SPI_CLOCK_DIV16); //5 MHz SPI Clock
   #elif (F_CPU == 120000000)
   SPI.setClockDivider(SPI_CLOCK_DIV16); //7.5 MHz SPI Clock
   #else
   #error "Radio SPI clock unable to be set < 10MHz"      
   #endif
  
   digitalWrite(CSPIN, LOW);
   
   uint32_t count=0;
   
   while(1){
     int v = digitalRead(SOPIN);
     count++;
     if(v == 0) break;
   }
  
  
   uint8_t cb = SPI.transfer(reg);
  
   digitalWrite(CSPIN, HIGH);
   
   SPI.end();
   
   p("%s[%02x] %s[%02x] count=%d\r\n",cc1101_cmd_str(reg),reg,cc1101_cb_str(cb),cb,count);
   
   return cb;
}

uint8_t regwrite(uint8_t reg,uint8_t value){

   SPI.begin();
   SPI.setModule(0);
   SPI.setDataMode(SPI_MODE0);
   
   #if (F_CPU == 1000000)
   SPI.setClockDivider(SPI_CLOCK_DIV1); //1 MHz SPI Clock  
   #elif (F_CPU == 16000000)
   SPI.setClockDivider(SPI_CLOCK_DIV2); //8 MHz SPI Clock
   #elif (F_CPU == 24000000)   
   SPI.setClockDivider(SPI_CLOCK_DIV4); //6 MHz SPI Clock
   #elif (F_CPU == 80000000)
   SPI.setClockDivider(SPI_CLOCK_DIV16); //5 MHz SPI Clock
   #elif (F_CPU == 120000000)
   SPI.setClockDivider(SPI_CLOCK_DIV16); //7.5 MHz SPI Clock
   #else
   #error "Radio SPI clock unable to be set < 10MHz"      
   #endif
   
  
   digitalWrite(CSPIN, LOW);
   
   int count=0;
   while(1){
     int v = digitalRead(SOPIN);
     count++;
     if(v == 0) break;
   }

   SPI.transfer(reg);
   SPI.transfer(value);

   digitalWrite(CSPIN, HIGH);

   SPI.end();

}


uint8_t  register_read(uint8_t reg){

   SPI.begin();
   SPI.setModule(0);
   SPI.setDataMode(SPI_MODE0);
   
   #if (F_CPU == 1000000)
   SPI.setClockDivider(SPI_CLOCK_DIV1); //1 MHz SPI Clock  
   #elif (F_CPU == 16000000)
   SPI.setClockDivider(SPI_CLOCK_DIV2); //8 MHz SPI Clock
   #elif (F_CPU == 24000000)   
   SPI.setClockDivider(SPI_CLOCK_DIV4); //6 MHz SPI Clock
   #elif (F_CPU == 80000000)
   SPI.setClockDivider(SPI_CLOCK_DIV16); //5 MHz SPI Clock
   #elif (F_CPU == 120000000)
   SPI.setClockDivider(SPI_CLOCK_DIV16); //7.5 MHz SPI Clock
   #else
   #error "Radio SPI clock unable to be set < 10MHz"      
   #endif
   
  
   digitalWrite(CSPIN, LOW);
   
   int count=0;
   while(1){
     int v = digitalRead(SOPIN);
     count++;
     if(v == 0) break;
   }
  
   uint8_t cb = SPI.transfer(reg + 0x80);
   uint8_t rv = SPI.transfer(0x0);
   //rv = SPI.transfer(0x0);
   //rv = SPI.transfer(0x0);
   //rv = SPI.transfer(0x0);
   //rv = SPI.transfer(0x0);
   
   digitalWrite(CSPIN, HIGH);

   SPI.end();
   
   p("reg[%02x]=%02x %s[%02x] count=%d\r\n",reg,rv,cc1101_cb_str(cb),cb,count);
   
   return rv;
}



void regdump(void){
  
  for(uint16_t reg=0x00; reg<=0x3f; reg++){
      register_read(reg);
  }
  
}

#pragma pack(1)
typedef union {
   struct {
     uint8_t regsrw[48];
     uint8_t regsro[14];
     uint8_t patable[8];
     uint8_t rxfifo[64];
     uint8_t txfifo[64];
   } mem;
   
    struct {

      struct {
      
     //0x00: IOCFG2 – GDO2 Output Pin Configuration 
      struct {
       uint8_t notused:1; //Not used
       uint8_t gdo2_inv:1; // Invert output, i.e. select active low (1) / high (0)
       uint8_t gdo2_cfg:6; // Default is CHP_RDYn (See Table 41 on page 62).
     } iocfg2;
     
     //0x01: IOCFG1 – GDO1 Output Pin Configuration
      struct {
       uint8_t gdo_ds:1; // Set high (1) or low (0) output drive strength on the GDO pins
       uint8_t gdo1_inv:1; // Invert output, i.e. select active low (1) / high (0)
       uint8_t gdo1_cfg:6; // Default is 3-state (See Table 41 on page 62).
     } iocfg1;
     
     //0x02: IOCFG0 – GDO0 Output Pin Configuration
      struct {
       uint8_t temp_sensor_enable: 1; // Enable analog temperature sensor. Write 0 in all other register bits when using temperature sensor.
       uint8_t gdo0_inv: 1; // Invert output, i.e. select active low (1) / high (0)
       uint8_t gfo0_cfg: 6; // Default is CLK_XOSC/192 (See Table 41 on page 62).
       //It is recommended to disable the clock output in initialization, in order to optimize RF performance.
     } iocfg0;
     
     
     //0x03: FIFOTHR – RX FIFO and TX FIFO Thresholds
     struct {
       uint8_t notused:1; //Not used
       uint8_t adc_retention:1;
       /*
0: TEST1 = 0x31 and TEST2= 0x88 when waking up from SLEEP
1: TEST1 = 0x35 and TEST2 = 0x81 when waking up from SLEEP
Note that the changes in the TEST registers due to the
ADC_RETENTION bit setting are only seen INTERNALLY in the analog
part. The values read from the TEST registers when waking up from
SLEEP mode will always be the reset value.
The ADC_RETENTION bit should be set to 1before going into SLEEP
mode if settings with an RX filter bandwidth below 325 kHz are wanted at
time of wake-up.   
       */
       uint8_t close_in_rx: 2;
/*
For more details, please see DN010 [8]
Setting RX Attenuation, Typical Values
0 (00) 0 dB
1 (01) 6 dB
2 (10) 12 dB
3 (11) 18 dB
*/       
       uint8_t fifo_thr : 4;
/*
Set the threshold for the TX FIFO and RX FIFO. The threshold is
exceeded when the number of bytes in the FIFO is equal to or higher than
the threshold value.
Setting Bytes in TX FIFO Bytes in RX FIFO
0 (0000) 61 4
1 (0001) 57 8
2 (0010) 53 12
3 (0011) 49 16
4 (0100) 45 20
5 (0101) 41 24
6 (0110) 37 28
7 (0111) 33 32
8 (1000) 29 36
9 (1001) 25 40
10 (1010) 21 44
11 (1011) 17 48
12 (1100) 13 52
13 (1101) 9 56
14 (1110) 5 60
15 (1111) 1 64
*/       
     } fifothr;
     
     //0x04: SYNC1 – Sync Word, High Byte
     uint8_t sync1; // 8 MSB of 16-bit sync word
     
     //0x05: SYNC0 – Sync Word, Low Byte
     uint8_t sync0; // 8 LSB of 16-bit sync word
     
     //0x06: PKTLEN – Packet Length 
     uint8_t pktlen; 
     /*
Indicates the packet length when fixed packet length mode is enabled. 
If variable packet length mode is used, this value indicates the maximum packet length allowed. 
This value must be different from 0.     
     */
     
     //0x07: PKTCTRL1 – Packet Automation Control
     struct {
       uint8_t pqt: 3;
       /*
       Preamble quality estimator threshold. The preamble quality estimator
increases an internal counter by one each time a bit is received that is
different from the previous bit, and decreases the counter by 8 each time a
bit is received that is the same as the last bit.
A threshold of 4 PQT for this counter is used to gate sync word detection.
When PQT=0 a sync word is always accepted.
       */ 
       uint8_t unused: 1;
       uint8_t crc_autoflush: 1; 
       /*
Enable automatic flush of RX FIFO when CRC is not OK. 
This requires that only one packet is in the RXIFIFO and that packet length is limited to the
RX FIFO size.       
       */
       uint8_t append_status: 1; 
/*
When enabled, two status bytes will be appended to the payload of the
packet. The status bytes contain RSSI and LQI values, as well as CRC OK.
*/       
      uint8_t adr_chk: 2;
/*
Controls address check configuration of received packages.
Setting Address check configuration
0 (00) No address check
1 (01) Address check, no broadcast
2 (10) Address check and 0 (0x00) broadcast
3 (11) Address check and 0 (0x00) and 255 (0xFF)
broadcast
*/       
     
     } pktctrl1;
     
     //0x08: PKTCTRL0 – Packet Automation Control
     struct {
       uint8_t unused1: 1;
       
       uint8_t white_data: 1;
       /*
     Turn data whitening on / off
0: Whitening off
1: Whitening on
     */
     
       uint8_t pkt_format: 2;
       /*
Format of RX and TX data
Setting Packet format
0 (00) Normal mode, use FIFOs for RX and TX
1 (01) Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins
2 (10) Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX
3 (11) Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins       
       */

       uint8_t unused2: 1;

       uint8_t crc_en: 1;
/*
1: CRC calculation in TX and CRC check in RX enabled
0: CRC disabled for TX and RX
*/       
       uint8_t length_config: 2;
/*
Configure the packet length
Setting Packet length configuration
0 (00) Fixed packet length mode. Length configured in PKTLEN register
1 (01) Variable packet length mode. Packet length configured by the first byte after sync word
2 (10) Infinite packet length mode
3 (11) Reserved
*/    
     } pktctrl0;
     
     //0x09: ADDR – Device Address 
     uint8_t addr;
     /*
Address used for packet filtration. Optional broadcast addresses are 0
(0x00) and 255 (0xFF).  
     */
     
     //0x0A: CHANNR – Channel Number 
     uint8_t channr;
     /*
 The 8-bit uint8_t channel number, which is multiplied by the channel
spacing setting and added to the base frequency
     */
     
     //0x0B: FSCTRL1 – Frequency Synthesizer Control
     struct {
       uint8_t unused : 2;
       uint8_t reserved : 1;
       uint8_t freq_if: 5;
       /*
The desired IF frequency to employ in RX. Subtracted from FS base frequency
in RX and controls the digital complex mixer in the demodulator.
FIF= FOSC / pow(2,10) * freq_if
The default value gives an IF frequency of 381kHz, assuming a 26.0 MHz crystal.
       */
       
     } fsctrl1;
     
     //0x0C: FSCTRL0 – Frequency Synthesizer Control
     uint8_t fsctrl0;
     /*
Frequency offset added to the base frequency before being used by the
frequency synthesizer. (2s-complement).
Resolution is FXTAL/214 (1.59kHz-1.65kHz); range is +-202 kHz to +-210 kHz,
dependent of XTAL frequency.     
     */
     
     //0x0D: FREQ2 – Frequency Control Word, High Byte
     //0x0E: FREQ1 – Frequency Control Word, Middle Byte
     //0x0F: FREQ0 – Frequency Control Word, Low Byte
     uint8_t freq2;
     uint8_t freq1;
     uint8_t freq0;
     /*
     FREQ[23:22] is always 0 (the FREQ2 register is less than 36 with 26-27 MHz crystal)
     FREQ[23:0] 
     FREQ[15:8] 
     FREQ[7:0] 
     is the base frequency for the frequency synthesiser in increments of fXOSC/pow(2,16)
     fcarrier=fosc/pow(2,16)*freq[23:0]
     */
          
     //0x10: MDMCFG4 – Modem Configuration
     struct {
       uint8_t chanbw_e: 2;
       uint8_t chanbw_m: 2;
       /*
       Sets the decimation ratio for the delta-sigma ADC input stream and thus the channel bandwidth.
       BW_channel = fosc / ( 8*(4+chanbw_m)*pow(2,chanbw_e);
       The default values give 203 kHz channel filter bandwidth, assuming a 26.0 MHz crystal.
       */
       
       uint8_t drate_e: 4; //The exponent of the user specified symbol rate
       
     } mdmcfg4;
     
     //0x11: MDMCFG3 – Modem Configuration
     struct {
       uint8_t drate_m: 8;
       /*
The mantissa of the user specified symbol rate. 
The symbol rate is configured using an unsigned, 
floating-point number with 9-bit mantissa and 4-bit exponent. 
The 9th bit is a hidden ‘1’. The resulting data rate is:
The default values give a data rate of 115.051 kBaud 
(closest setting to 115.2 kBaud), assuming a 26.0 MHz crystal.       
       */
     } mdmcfg3;
     
     
     //0x12: MDMCFG2 – Modem Configuration
     uint8_t mdmcfg2;
     
     //0x13: MDMCFG1– Modem Configuration
     uint8_t mdmcfg1;
     
     //0x14: MDMCFG0– Modem Configuration
     uint8_t mdmcfg0;
     
     //0x15: DEVIATN – Modem Deviation Setting
     uint8_t deviatn;
     
     //0x16: MCSM2 – Main Radio Control State Machine Configuration
     uint8_t mcsm2;
     
     //0x17: MCSM1– Main Radio Control State Machine Configuration
     uint8_t mcsm1;
     
     //0x18: MCSM0– Main Radio Control State Machine Configuration
     uint8_t mcsm0;
     
     //0x19: FOCCFG – Frequency Offset Compensation Configuration
     uint8_t foccfg;
     
     //0x1A: BSCFG – Bit Synchronization Configuration
     uint8_t bscfg;
     
     //0x1B: AGCCTRL2 – AGC Control 
     uint8_t agcctrl2;
     
     //0x1C: AGCCTRL1 – AGC Control 
     uint8_t agcctrl1;
     
     //0x1D: AGCCTRL0 – AGC Control 
     uint8_t agcctrl0;
     
     //0x1E: WOREVT1 – High Byte Event0 Timeout 
     uint8_t worevet1;
     
     //0x1F: WOREVT0 –Low Byte Event0 Timeout 
     uint8_t worevet0;
     
     //0x20: WORCTRL – Wake On Radio Control 
     uint8_t worctrl;
     
     //0x21: FREND1 – Front End RX Configuration
     uint8_t frend1;
     
     //0x22: FREND0 – Front End TX Configuration
     uint8_t frend0;
     
     //0x23: FSCAL3 – Frequency Synthesizer Calibration
     uint8_t fscal3;
     
     //0x24: FSCAL2 – Frequency Synthesizer Calibration
     uint8_t fscal2;
     
     //0x25: FSCAL1 – Frequency Synthesizer Calibration 
     uint8_t fscal1;
     
     //0x26: FSCAL0 – Frequency Synthesizer Calibration
     uint8_t fscal0;
     
     //0x27: RCCTRL1 – RC Oscillator Configuration
     uint8_t rcctrl1;
     
     //0x28: RCCTRL0 – RC Oscillator Configuration
     uint8_t rcctrl0;
     
     //0x29: FSTEST – Frequency Synthesizer Calibration Control
     uint8_t fstest;
     
     //0x2A: PTEST – Production Test 
     uint8_t ptest;
     
     //0x2B: AGCTEST – AGC Test 
     uint8_t agctest;
     
     //0x2C: TEST2 – Various Test Settings
     uint8_t test2;
     
     //0x2D: TEST1 – Various Test Settings
     uint8_t test1;
     
     //0x2E: TEST0 – Various Test Settings
     uint8_t test0;
     
     //0x2F
     uint8_t pad;
    
     } regsrw; 

     struct {
       
     //0x30 (0xF0): PARTNUM – Chip ID 
     uint8_t partmun;
     
     //0x31 (0xF1): VERSION – Chip ID 
     uint8_t version;
     
     //0x32 (0xF2): FREQEST – Frequency Offset Estimate from Demodulator
     uint8_t freqest;
     
     //0x33 (0xF3): LQI – Demodulator Estimate for Link Quality
     uint8_t lqi;
     
     //0x34 (0xF4): RSSI – Received Signal Strength Indication
     uint8_t rssi;
     
     //0x35 (0xF5): MARCSTATE – Main Radio Control State Machine State
     uint8_t marcstate;
     
     //0x36 (0xF6): WORTIME1 – High Byte of WOR Time
     uint8_t wortime1;
     
     //0x37 (0xF7): WORTIME0 – Low Byte of WOR Time
     uint8_t wortime0;
     
     //0x38 (0xF8): PKTSTATUS – Current GDOx Status and Packet Status
     uint8_t pktstatus;
     
     //0x39 (0xF9): VCO_VC_DAC – Current Setting from PLL Calibration Module
     uint8_t vc0_vc_dac;
     
     //0x3A (0xFA): TXBYTES – Underflow and Number of Bytes
     uint8_t txbytes;
     
     //0x3B (0xFB): RXBYTES – Overflow and Number of Bytes
     uint8_t rxbytes;
     
     //0x3C (0xFC): RCCTRL1_STATUS – Last RC Oscillator Calibration Result
     uint8_t rcctrl1_status;
     
     //0x3D (0xFD): RCCTRL0_STATUS – Last RC Oscillator Calibration Result
     uint8_t rcctrl0_status;
     
     } regsro;
     
     //0x3E (PATABLE)
     uint8_t patable[8];
     
     //0x3F FIFO
     uint8_t rxfifo[64];
     
     //0x3F FIFO
     uint8_t txfifo[64];
   } view;

} radiomem_t ;
#pragma pack()




radiomem_t radio;

uint8_t pktlen=9;
void loop()
{

    SPI.begin();
    SPI.setModule(2);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);    
    digitalWrite(CSPIN, LOW);
    int cb=SPI.transfer(0x06);
    int val=SPI.transfer(pktlen);
    digitalWrite(CSPIN, HIGH);
    SPI.end();
    
    pktlen++;
 
   int ooo=0;
  
   p("[");
   for(int reg=0x0;reg<=0x2f;reg++){
    SPI.begin();
    SPI.setModule(2);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);    
    digitalWrite(CSPIN, LOW);
    int cb=SPI.transfer(reg + 0xc0);
    int val=SPI.transfer(0);
    digitalWrite(CSPIN, HIGH);
    SPI.end();
    
    radio.mem.regsrw[reg]=val;
    
    p("%02x",val);
    ooo++;
   }
   p("][");
   
   int regro=0;
   for(int reg=0x30;reg<=0x3d;reg++){
    SPI.begin();
    SPI.setModule(2);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);    
    digitalWrite(CSPIN, LOW);
    int cb=SPI.transfer(reg + 0xc0);
    int val=SPI.transfer(0);
    digitalWrite(CSPIN, HIGH);
    SPI.end();
    
    radio.mem.regsro[regro++]=val;
    
    p("%02x",val);
    ooo++;
   }
   p("][");
   
    SPI.begin();
    SPI.setModule(2);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);    
    digitalWrite(CSPIN, LOW);
    SPI.transfer(0x3e + 0xc0);
    for(int k=0;k<8;k++){
     int val=SPI.transfer(0);
     
     radio.mem.patable[k]=val;
     
     p("%02x",val);
     ooo++;
    }
    digitalWrite(CSPIN, HIGH);
    SPI.end();

   p("][");
   
    SPI.begin();
    SPI.setModule(2);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);    
    digitalWrite(CSPIN, LOW);
    SPI.transfer(0x3f + 0xc0);
    for(int k=0;k<64;k++){
     int val=SPI.transfer(0);
     
     radio.mem.rxfifo[k]=val;
     
     p("%02x",val);
     ooo++;
    }
    digitalWrite(CSPIN, HIGH);
    SPI.end();
   
   p("]=%d\r\n",ooo);
}




