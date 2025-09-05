#include <Wire.h>
#include <elapsedMillis.h>
#include <Comparator.h>

uint16_t readDataNum; //store the number of read data
uint16_t startRegAddress; //the start address to write or read from
uint8_t tempData[396]; //store the data to write
uint16_t adc; //General adc value variable
uint8_t sensor_baseline; //The baseline voltage the photodiode is at when there is only background illumination
const int baseline_offset = 2; //How much to subtract from the measured baseline to stop false triggering
elapsedMicros interrupt_timer; //Timer for duration of sensor interrupt
elapsedMillis timer;  //General ms timer for messages and timeouts
elapsedMicros timeout_timer; //Timer for checking if communication has timed out
const uint16_t comm_timeout = 50000; //How many milliseconds to wait for a full data stream to complete before timing out 
volatile uint32_t interrupt_duration; //How long the comparator was low (IR LED on)
volatile uint8_t sensor_state; //WHether the comparator output is high or low
uint16_t led_on_duration; //How long an IR LED pulse is to indicate an LED is to be turned on
const uint16_t start_com_duration = 200;
//I2C
#define i2cWrite 0x00
#define i2cRead 0x01
#define vsync_pin 10
#define sensor 4
#define output_pin 1

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

void ac_interrupt() //Comparator interrupt
{
  if (Comparator0.read()){ //When comparator is high
    sensor_state = 1;
    interrupt_duration = interrupt_timer;
  }
  else{ //When comparator is low
    sensor_state = 2;
    interrupt_timer = 0;
  }
}

// ISR(PORTA_PORT_vect) { //Output pin interrupt
//   if (PORTA.INTFLAGS & PIN5_bm) {
//     bool current_state = (PORTA.IN & PIN5_bm);
//     if (current_state) interrupt_duration = interrupt_timer; //When pin is high
//     else interrupt_timer = 0; //Reset interrupt timer //When pin is low
//     PORTA.INTFLAGS = PIN5_bm; // Clear the interrupt flag for PB3
//   }
// }

void setup() {
  uint8_t i;

  //Initialize I2C
  pinMode(6, INPUT); //Pin 6 is connected to Gnd to set it to input
  pinMode(SDA, INPUT_PULLUP); //Needed to use internal pullups as I2C pullup
  pinMode(SCL, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(1e6);
  Wire.usePullups();

  //Get sensor baseline
  sensor_baseline = measureSensorBaseline();

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
  tempData[3] = B11110001; //data to Dev_config3 register, weak down deghost (default), vled-2v up deghost (default), 15mA maximum current (default), enable up deghost (default)
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
  //rain();
  //counterChase();
  //sensorMeter();
  //zigzag();
  //comparatorTest();
  pinMode(sensor, INPUT);
  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0x5F;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0
  startComparator();
}

void loop() {
  uint16_t i, j;
  uint8_t display[24];
  uint8_t nec_byte[16];
  bool nec_valid;

  while(sensor_state != 2); //Wait for the start of a communication
  while(sensor_state != 1 || interrupt_duration < start_com_duration); //Wait for LED to go low again
  interrupt_duration = 0;
  while(!interrupt_duration); //Wait for first timing pulse
  led_on_duration = interrupt_duration>>1; //Record the duration of the first pulse, as this indicated the LED-on duration
  for(i=0; i<24; i++) display[i] = 0; //Zero out the display array
  timeout_timer = 0;
  interrupt_duration = 0;
  for(i=0; i<n_rows*2 && timeout_timer < comm_timeout; i++){ //2 bytes per row for NEC IR encoding
    nec_byte[i] = 0;
    for(j=0; j<8 && timeout_timer < comm_timeout;){
      if(sensor_state == 1 && interrupt_duration){
        if(interrupt_duration > led_on_duration) nec_byte[i] += 1 << j;
        interrupt_duration = 0; //Reset interrupt duration
        timeout_timer = 0; //Reset timout timer
        j++; //Increment index
      }
    }
  }
  nec_valid = true;
  for(i=0; i<n_rows*2 && nec_valid; i+=2){ //Verify that NEC encoding is valid
    nec_byte[i+1] ^= nec_byte[i];
    if(nec_byte[i+1] != 0xFF) nec_valid = false;
  }
  if(i<n_rows*2 || j<8 || !nec_valid){ //If an error happened during communication
    for(i=0; i<24; i++) display[i] = 0; //Zero out the display array
    // for(i=0; i<n_rows; i++){
    //   uint16Union.bytes_var = 0;
    //   for(j=0; j<8; j++){
    //     if(nec_byte[i] & 1 << j) uint16Union.bytes_var += 1 << led_order[j];
    //   }
    //   display[i*3] = uint16Union.bytes[0]; //shift display down one row
    //   display[i*3+1] = uint16Union.bytes[1]; //shift display down one row
    // }
    display[1] = 1; //Turn on red error LED
  }
  else{
    for(i=0; i<n_rows; i++){
      uint16Union.bytes_var = 0;
      for(j=0; j<8; j++){
        if(nec_byte[i*2] & 1 << j) uint16Union.bytes_var += 1 << led_order[j];
      }
      display[i*3] = uint16Union.bytes[0]; //shift display down one row
      display[i*3+1] = uint16Union.bytes[1]; //shift display down one row
    } 
  }
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
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
///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM///////////////////nIR COMM
uint8_t measureSensorBaseline(){
  uint8_t i;
  adc = 0;
  for(i=0; i<8; i++){
    adc += analogRead(sensor);
    delay(100);
  }
  adc >>= 5;
  return adc; 
}

//https://github.com/SpenceKonde/megaTinyCore/tree/master/megaavr/libraries/Comparator
//https://github.com/grughuhler/attiny/blob/main/attiny_ac/attiny_ac.ino
void startComparator(){
  sensor_baseline = 255;
  pinMode(output_pin, OUTPUT);
  Comparator.input_p = comparator::in_p::in1;       // pos input PA7.  See datasheet
  Comparator.input_n = comparator::in_n::dacref;    // neg pin to the DACREF voltage
  Comparator.reference = comparator::ref::vref_vdd; // Set the DACREF voltage
  Comparator.dacref = sensor_baseline;

  Comparator.hysteresis = comparator::hyst::large;  // Use 50mV hysteresis
  Comparator.output = comparator::out::enable;      // Enable output PB3
  Comparator.output_initval = comparator::out::init_high; // Output pin high after initialization
  Comparator.attachInterrupt(ac_interrupt, CHANGE);
  Comparator.init();
  Comparator.start();
  while(!Comparator0.read()){
    AC0.DACREF = sensor_baseline--;
    delay(10);
  }
  sensor_baseline -= baseline_offset;
  AC0.DACREF = sensor_baseline;
  sensor_state = 1;  //Set state to active
}

void stopComparator(){
  Comparator.detachInterrupt();
  Comparator.stop(true); // Stop comparator. Digital input on the pins that this comparator was using will be re-enabled.
  sensor_state = 0; //Set state to standby
}


///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS///////////////////PATTERNS
#define wordsNum 8  //8 words to display in wordRollingPlayback pattern
#define wordsRow 16 //10 rows of LED dots to display the words
#define wordsCol 8 //6 cols of LED dots to display the words

void comparatorTest(){
  int i, j, k;
  uint16_t counter;
  uint16_t mask;
  uint8_t display[24];
  uint8_t duration;
  
  pinMode(sensor, INPUT);
  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0xFF;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0
  for(i=0; i<24; i++) display[i] = 0;
  startComparator();
  while(true){
    if(sensor_state == 1){
      duration = interrupt_duration>>2;
      counter = 0;
      for(i=0; i<8; i++){
        uint16Union.bytes_var = 0;
        for(j=0; j<8; j++){
          if(counter == duration) uint16Union.bytes_var += 1 << led_order[j];
          counter++;
        } 
        display[i*3] = uint16Union.bytes[0]; //shift display down one row
        display[i*3+1] = uint16Union.bytes[1]; //shift display down one row
      }
    }
    else{
      for(i=0; i<24; i++) display[i] = 0;
      display[0] = 256;
    }
    i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
  }

}

void rain(){
  int i, j, k;
  uint8_t display[24];
  const uint8_t inv_density = 9;

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

void counterChase(){
  uint16_t adc;
  int i, j, k;
  uint8_t counter;
  uint8_t display[24];

  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0xFF;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0

  while(true){
    // adc = analogRead(sensor);
    // adc >>= 6;
    // adc = 64-adc;
    adc++;
    if(adc > 64) adc = 0;
    counter = 0;
    for(i=0; i<8; i++){
      uint16Union.bytes_var = 0;
      for(j=0; j<8; j++){
        if(counter == adc) uint16Union.bytes_var += 1 << led_order[j];
        counter++;
      } 
      display[i*3] = uint16Union.bytes[0]; //shift display down one row
      display[i*3+1] = uint16Union.bytes[1]; //shift display down one row
    }
    i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
    delay(100); 
  }
}

void sensorMeter(){
  uint16_t adc;
  int i, j, k;
  uint16_t counter;
  uint16_t mask;
  uint8_t display[24];
  const uint8_t inv_density = 9;
  bool debug = false;

  pinMode(sensor, INPUT);
  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0xFF;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0
  for(i=0; i<24; i++) display[i] = 0;
  while(true){
    adc = analogRead(sensor);
    adc>>=2;
    adc -= 170;
    adc = 64-adc;
    counter = 0;
    for(i=0; i<8; i++){
      uint16Union.bytes_var = 0;
      for(j=0; j<8; j++){
        if(counter <= adc) uint16Union.bytes_var += 1 << led_order[j];
        counter++;
      } 
      display[i*3] = uint16Union.bytes[0]; //shift display down one row
      display[i*3+1] = uint16Union.bytes[1]; //shift display down one row
    }

    //Show binary value
    // for(i=0; i<2; i++){
    //   uint16Union.bytes_var = 0;
    //   for(j=0; j<8; j++){
    //     mask = 1 << j + i*8;
    //     if(adc & mask) uint16Union.bytes_var += 1 << led_order[j];
    //   } 
    //   display[i*3] = uint16Union.bytes[0]; //shift display down one row
    //   display[i*3+1] = uint16Union.bytes[1]; //shift display down one row
    // }
    i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
    delay(100); 
  }
}
