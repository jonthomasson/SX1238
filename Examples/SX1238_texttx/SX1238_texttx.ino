
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)

// **********************************************************************************
// SX1238 Internal registers addresses
//**************************************************
#define REG_FIFO          0x00
#define REG_OPMODE        0x01
#define REG_BITRATEMSB    0x02
#define REG_BITRATELSB    0x03
#define REG_FDEVMSB       0x04
#define REG_FDEVLSB       0x05
#define REG_FRFMSB        0x06
#define REG_FRFMID        0x07
#define REG_FRFLSB        0x08
#define REG_PACONFIG      0x09
#define REG_PARAMP        0x0A
#define REG_OCP           0x0B
#define REG_LNA           0x0C
#define REG_RXCONFIG      0x0D
#define REG_RSSICONFIG    0x0E
#define REG_RSSICOLLISION 0x0F
#define REG_RSSITHRESH    0x10
#define REG_RSSIVALUE     0x11
#define REG_RXBW          0x12
#define REG_AFCBW         0x13
#define REG_OOKPEAK       0x14
#define REG_OOKFIX        0x15
#define REG_OOKAVG        0x16
#define REG_AFCFEI        0x1A
#define REG_AFCMSB        0x1B
#define REG_AFCLSB        0x1C
#define REG_FEIMSB        0x1D
#define REG_FEILSB        0x1E
#define REG_PREAMBLEDETECT  0x1F
#define REG_RXTIMEOUT1    0x20
#define REG_RXTIMEOUT2    0x21
#define REG_RXTIMEOUT3    0x22
#define REG_RXDELAY       0x23
#define REG_OSC           0x24
#define REG_PREAMBLEMSB   0x25
#define REG_PREAMBLELSB   0x26
#define REG_SYNCCONFIG    0x27
#define REG_SYNCVALUE1    0x28
#define REG_SYNCVALUE2    0x29
#define REG_SYNCVALUE3    0x2A
#define REG_SYNCVALUE4    0x2B
#define REG_SYNCVALUE5    0x2C
#define REG_SYNCVALUE6    0x2D
#define REG_SYNCVALUE7    0x2E
#define REG_SYNCVALUE8    0x2F
#define REG_PACKETCONFIG1 0x30
#define REG_PACKETCONFIG2 0x31
#define REG_PAYLOADLENGTH 0x32
#define REG_NODEADRS      0x33
#define REG_BROADCASTADRS 0x34
#define REG_FIFOTHRESH    0x35
#define REG_SEQCONFIG1    0x36
#define REG_SEQCONFIG2    0x37
#define REG_TIMERRESOL    0x38
#define REG_TIMER1COEF    0x39
#define REG_TIMER2COEF    0x3A
#define REG_IMAGECAL      0x3B
#define REG_TEMP          0x3C
#define REG_LOWBAT        0x3D
#define REG_IRQFLAGS1     0x3E
#define REG_IRQFLAGS2     0x3F
#define REG_DIOMAPPING1   0x40
#define REG_DIOMAPPING2   0x41
#define REG_VERSION       0x42
#define REG_AGCREF        0x43
#define REG_AGCTHRESH1    0x44
#define REG_AGCTHRESH2    0x45
#define REG_AGCTHRESH3    0x46
#define REG_TCXO          0x58
#define REG_PADAC         0x5A
#define REG_PLL           0x5C
#define REG_PLLLOWPN      0x5E
#define REG_FORMERTEMP    0x6C
#define REG_BITRATEFRAC   0x70

#define SERIAL_BAUD       115200
#define SS                10
#define TX_EN             11
#define RX_EN             12
#define MODE              13

// RegIrqFlags1
#define RF_IRQFLAGS1_MODEREADY            0x80
#define RF_IRQFLAGS1_RXREADY              0x40
#define RF_IRQFLAGS1_TXREADY              0x20
#define RF_IRQFLAGS1_PLLLOCK              0x10
#define RF_IRQFLAGS1_RSSI                 0x08
#define RF_IRQFLAGS1_TIMEOUT              0x04
#define RF_IRQFLAGS1_PREAMBLEDETECT       0x02
#define RF_IRQFLAGS1_SYNCADDRESSMATCH     0x01

//transceiver modes
#define SX1238_MODE_SLEEP         0 //none
#define SX1238_MODE_STANDBY       1 // Top regulator and crystal oscillator
#define SX1238_MODE_SYNTH_TX      2 // Frequency synthesizer at Tx frequency (Frf)
#define SX1238_MODE_TX            3 // Frequency synthesizer and transmitter
#define SX1238_MODE_SYNTH_RX      4 // Frequency synthesizer at frequency for reception (Frf-IF)
#define SX1238_MODE_RX            5 // Frequency synthesizer and receiver

//public
static volatile uint8_t _mode;

void setup() {
  //setup serial
  Serial.begin(SERIAL_BAUD);

  //setup cs pin
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);

  //setup spi
  SPI.begin();

  //setup registers for transmitting
  

}

//reads the register value of supplied address
uint8_t readRegister(uint8_t addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  unselect();
  return regval;
}

//writes register value to supplied address
void writeRegister(uint8_t addr, uint8_t value)
{
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

// select the transceiver (save SPI settings, set CS low)
void select() {
 
  // set SPI settings
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); //2MHz clock, MSB first
  digitalWrite(SS, LOW);
}

// unselect the transceiver (set CS high)
void unselect() {
  digitalWrite(SS, HIGH);

}

void loop(){
  //transmit once a second
  delay(1000);  
  transmitSomething();  
}

void setMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  writeRegister(REG_OPMODE, newMode); 
//  switch (newMode) {
//    case RF69_MODE_TX:
//      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
//      if (_isRFM69HW) setHighPowerRegs(true);
//      break;
//    case RF69_MODE_RX:
//      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
//      if (_isRFM69HW) setHighPowerRegs(false);
//      break;
//    case RF69_MODE_SYNTH:
//      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
//      break;
//    case RF69_MODE_STANDBY:
//      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
//      break;
//    case RF69_MODE_SLEEP:
//      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
//      break;
//    default:
//      return;
  }

//transmit a packet of data
void transmitSomething(){
  Serial.println("setting REG_OPMODE to tx mode 3");
  setMode(SX1238_MODE_TX); //set mode to transmit
  Serial.print("REG_OPMODE = ");
  Serial.println(readRegister(REG_OPMODE));

  digitalWrite(RX_EN, LOW); //disable rx
  digitalWrite(MODE, LOW);
  digitalWrite(TX_EN, HIGH); //enable power amp transmitter

  writeRegister(REG_PACONFIG, 1); //output power to 1

  writeRegister(REG_PACKETCONFIG1, 128); //turn off crc

  //transmit packets
  
  
}

void sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
  setMode(SX1238_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // control byte
  uint8_t CTLbyte = 0x00;
  if (sendACK)
    CTLbyte = RFM69_CTL_SENDACK;
  else if (requestACK)
    CTLbyte = RFM69_CTL_REQACK;

  // write to FIFO
  select();
  SPI.transfer(REG_FIFO | 0x80);
  SPI.transfer(bufferSize + 3);
  SPI.transfer(toAddress);
  SPI.transfer(_address);
  SPI.transfer(CTLbyte);

  for (uint8_t i = 0; i < bufferSize; i++)
    SPI.transfer(((uint8_t*) buffer)[i]);
  unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
  uint32_t txStart = millis();
  while (digitalRead(_interruptPin) == 0 && millis() - txStart < RF69_TX_LIMIT_MS); // wait for DIO0 to turn HIGH signalling transmission finish
  //while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
  setMode(RF69_MODE_STANDBY);
}



