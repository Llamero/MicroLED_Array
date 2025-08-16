#include <Wire.h>

uint16_t readDataNum; //store the number of read data
uint16_t startRegAddress; //the start address to write or read from
uint16_t i;
uint8_t tempData[396]; //store the data to write

//I2C
#define i2cWrite 0x00
#define i2cRead 0x01
#define i2c_pullup_pin 7
#define vsync_pin 5

/* Data sent to the Target */
uint8_t gTxPacket[396];

/* Data received from Target */
uint8_t gRxPacket[396];
/* define register address*/

/* I2C Target address */
#define I2C_TARGET_ADDRESS_INDEPENDENT (0x10) //b0001 0000
#define I2C_TARGET_ADDRESS_BROADCAST (0x15) //0001 0101

#define chip_en_reg     (0x000)
#define dev_initial     (0x001)
#define dev_config2     (0x003)
#define dev_config3     (0x004)
#define global_bri      (0x005)
#define group0_bri      (0x006)
#define dot_grp_sel0    (0x00C)
#define dot_onoff0      (0x043)
#define dot_lsd30       (0x0A4)
#define lsd_clear       (0x0A8)
#define reset_reg       (0x0A9)
#define dc0             (0x100)
#define pwm_bri0        (0x200)
#define pwm_bri180      (0x2B4)

#define n_rows          (8)
#define n_cols          (18)
#define n_leds          (n_rows*n_cols) //Number to total possible LEDs in array - 8x18

/* Indicates status of I2C */
enum I2cControllerStatus {
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
} gI2cControllerStatus;

/* Data sent to the Target */
extern uint8_t gTxPacket[];

/* Data received from Target */
extern uint8_t gRxPacket[];

/* Counters for TX length and bytes sent */
uint16_t gTxLen, gTxCount;

/* Counters for TX length and bytes sent */
uint16_t gRxLen, gRxCount;

/**
 *  @brief      write or read data through i2c
 *
 *  @param[in]  i2cTargetAddress  i2c slave address
 *  @param[in]  startRegAddress  start register address to write or read
 *  @param[in]  writeOrRead  write or read operation
 *  @param[in]  dataLength data length of the data to write or read
 *  @param[in]  data  point to the data array to write
 */
const uint8_t led_order[] = {0, 15, 2, 13, 4, 11, 6, 9};

union BYTE16UNION
{
 uint16_t bytes_var;
 uint8_t bytes[2];
}uint16Union;

void setup() {
  Serial.begin(9600);
  pinMode(i2c_pullup_pin, OUTPUT);
  digitalWrite(i2c_pullup_pin, HIGH);
  pinMode(vsync_pin, OUTPUT);
  digitalWrite(vsync_pin, LOW);
  Wire.begin();
  delay(1);

  /* reset device */
  startRegAddress = reset_reg;
  tempData[0] = 0xFF; //data to reset register
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, startRegAddress, i2cWrite, 1, &tempData[0]);
  delayMicroseconds(500); //wait for t_por (max 500us) to enter normal mode
  /* enable device */
  startRegAddress = chip_en_reg;
  tempData[0] = 0x01; //data to Chip_en register, enable chip
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, startRegAddress, i2cWrite, 1, &tempData[0]);
  delayMicroseconds(100); //wait for t_chip_en (max 100us) to enter normal mode
  /* initialize device */
  startRegAddress = dev_initial;
  tempData[0] = B01111000; //data to Dev_initial register, 11 max_line_num (default), set mode 1, 125kHz pwm_fre (default)
  tempData[1] = B00000100; //data to Dev_config1 register, 1us sw_blk (default), enable exponential scale dimming curve, phase shift off (default), cs_on_shift off (default)
  tempData[2] = B00000001; //data to Dev_config2 register, comp_group3/2/1 off (default), lod_removal disable (defualt), enable lsd_removal
  tempData[3] = B11110011; //data to Dev_config3 register, weak down deghost (default), vled-2v up deghost (default), 15mA maximum current (default), enable up deghost (default)
                           // Current: 000 = 7.5 mA, 001 = 12.5 mA, 010 = 25 mA, 011 = 37.5 mA, 100 = 50 mA, 101 = 75 mA, 110 = 100 mA

  //Turn off deghost
  // tempData[0] = B00100000; //data to Dev_initial register, 11 max_line_num (default), set mode 1, 125kHz pwm_fre (default)
  // tempData[1] = B00000100; //data to Dev_config1 register, 1us sw_blk (default), enable exponential scale dimming curve, phase shift off (default), cs_on_shift off (default)
  // tempData[2] = B00000000; //data to Dev_config2 register, comp_group3/2/1 off (default), lod_removal disable (defualt), enable lsd_removal
  // tempData[3] = B00000000; //data to Dev_config3 register, weak down deghost (default), vled-2v up deghost (default), 15mA maximum current (default), enable up deghost (default)
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, startRegAddress, i2cWrite, 4, &tempData[0]);
  /* read example */
  //i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, chip_en_reg, i2cRead, 5, &tempData[0]); //e.g. read 5 bytes data from chip_en_reg register
  //setValuesOverSerial();
  rain();
  //zigzag();
}

void loop() {
}

///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C///////////////////I2C

uint16_t i2cSendReceive(uint8_t i2cTargetAddress, uint16_t startRegAddress, uint8_t writeOrRead, uint16_t dataLength, uint8_t *data)
{
    uint16_t i,j,k;

    i2cTargetAddress = (i2cTargetAddress << 2) | ((uint8_t) (startRegAddress >> 8)); //get the highest 2 bits of the
                                                                                    //startRegAddress to construct the i2c slave address
                                                                                    //with the 5 bits device address
    gTxPacket[0] = (uint8_t) startRegAddress; //get the lower 8 bits of the startRegAddress
    gRxCount = 0;

    if(writeOrRead == i2cWrite) //i2c write
    {
        gTxLen = dataLength + 1; //take the startRegAddress into account
        for(i=0; i<dataLength; i++)
        {
            gTxPacket[i+1] = data[i]; //store the data
        }

        Wire.beginTransmission(i2cTargetAddress);
        Wire.write(gTxPacket, dataLength+1);
        Wire.endTransmission(); 
    }
    else //i2c read
        /*
         * The Controller will first send the Start Condition, I2C Address with R/W bit set to write,
         * then do a repeated start + read to read the gRxLen number of datas
         * the read data stored in gRxPacket[]
         */
    {
        Wire.beginTransmission(i2cTargetAddress);
        Wire.write(gTxPacket[0]);
        Wire.endTransmission(); 
        Wire.requestFrom(i2cTargetAddress, dataLength);

        i=0;
        while (Wire.available()) { // slave may send less than requested
          gRxPacket[i++] = Wire.read(); // receive a byte as character
        }
        gRxCount = i;
    }
    return gRxCount;
}

///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS
#define wordsNum 8  //8 words to display in wordRollingPlayback pattern
#define wordsRow 16 //10 rows of LED dots to display the words
#define wordsCol 8 //6 cols of LED dots to display the words

void zigzag(){
  int i, j, k;
  uint8_t first_row;
  uint8_t display[24];
  bool dir = false;

  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0xFF;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0
  k=7;
  while(true){
    for(i=7; i>0; i--){
      for(j=2; j>=0; j--) display[i*3+j] = display[(i-1)*3+j]; //shift display down one row
    } 
    uint16Union.bytes_var = 1 << led_order[k];
    display[0] = uint16Union.bytes[0];
    display[1] = uint16Union.bytes[1];
    i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
    delay(30);
    if(dir){
      if(k==7){
        dir = false;
        k--;
      }
      else k++;
    }
    else{
      if(k==0){
        dir = true;
        k++;
      }
      else k--;
    }
  }
}

void rain(){
  int i, j, k;
  uint8_t first_row;
  uint8_t display[24];
  const uint8_t inv_density = 20;

  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0xFF;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0

  while(true){
    for(i=7; i>0; i--){
      for(j=2; j>=0; j--) display[i*3+j] = display[(i-1)*3+j]; //shift display down one row
    } 
    uint16Union.bytes_var = 0;
    for(i=7; i>=0; i--){
        if(!random(inv_density)) uint16Union.bytes_var += 1 << led_order[i];
    }
    display[0] = uint16Union.bytes[0];
    display[1] = uint16Union.bytes[1];
    i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
    delay(20);
  }
}


void setValuesOverSerial()
{
  uint16_t i, j, k;
  uint16_t coord[] = {0,0};
  //Set global brightness to 0
  //tempData[0] = 0;
  //i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT,global_bri, i2cWrite, 1, &tempData[0]);
  //Set all individual PWM to 255
  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0xFF;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0

  while(true){
    if(Serial.available()){
      i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0+coord[0], i2cWrite, 2, 0);
      recvWithEndMarker(coord); //Get new coordinate
      coord[0] *= 3; //The rows increment in blocks of 3 bytes
      uint16Union.bytes_var = 1 << led_order[coord[1]]; //Shift the bit to the correct column
      i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0+coord[0], i2cWrite, 2, uint16Union.bytes);
    } 
  }
}

void recvWithEndMarker(uint16_t *result_array) {
    uint8_t ndx = 0;
    uint8_t result_index = 0;
    char endMarker = '\n';
    char rc;
    const byte numChars = 32;
    char receivedChars[numChars];   // an array to store the received data
    
    while (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker && rc != ',' && rc != ' ') {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            result_array[result_index] = atoi(receivedChars);
            ndx = 0;
            result_index++;
            if (rc == endMarker) return;
        }
    }
}