//This code can be run as low as 8 MHz to reduce power consumption

#include <Wire.h>
#include <elapsedMillis.h>
#include <Comparator.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>

uint16_t readDataNum; //store the number of read data
uint16_t startRegAddress; //the start address to write or read from
uint8_t tempData[396]; //store the data to write
uint8_t display[24]; //store the display bytes
uint16_t adc; //General adc value variable
uint8_t sensor_baseline; //The baseline voltage the photodiode is at when there is only background illumination
const int baseline_offset = 2; //How much to subtract from the measured baseline to stop false triggering
elapsedMicros interrupt_timer; //Timer for duration of sensor interrupt
elapsedMillis timer;  //General ms timer for messages and timeouts
elapsedMicros timeout_timer; //Timer for checking if communication has timed out
const uint32_t wake_timeout = 3e6; //How many microseconds to wait for a wake command to complete before timing out 
const uint16_t comm_timeout = 1000; //How many microseconds to wait for a full data stream to complete before timing out 
volatile uint32_t interrupt_duration; //How long the comparator was low (IR LED on)
volatile uint8_t sensor_state; //WHether the comparator output is high or low
uint16_t led_on_duration; //How long an IR LED pulse is to indicate an LED is to be turned on
const uint16_t start_com_duration = 200; //How many microseonds a minimum start pulse is
const uint32_t power_down_duration = 500000; //How many microseonds a minimum power down pulse is
uint8_t power_down = 1; //Whether the device should be powered down to save the battery - 0 - wake, 1 = powering down, 2 = powered down
uint8_t pit_counter; //Number of PIT interrupts that have happened
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
  AC0.STATUS = AC_CMP_bm; //Clear the interrupt flag
  if (Comparator0.read()){ //When comparator is high
    sensor_state = 1;
    interrupt_duration = interrupt_timer;
  }
  else{ //When comparator is low
    sensor_state = 2;
    interrupt_timer = 0;
  }
}

ISR(RTC_PIT_vect) {
    // Clear PIT interrupt flag
    RTC.PITINTFLAGS = RTC_PI_bm;
}

void setup() {
  uint8_t i;

  sei();  // Enable global interrupts
  
  // Disable ADC, USART, SPI, internal timers, RTC, and watchdog timer since they aren't used
  ADC0.CTRLA &= ~ADC_ENABLE_bm;
  USART0.CTRLA = 0;
  USART1.CTRLA = 0;
  SPI0.CTRLA = 0;
  WDT.CTRLA = 0;
  TCA0.SINGLE.CTRLA &= ~(1 << TCA_SINGLE_ENABLE_bp); // Disable Timer/Counter Type A (TCA0)
  TCB0.CTRLA &= ~(1 << TCB_ENABLE_bp);
  CLKCTRL.OSC32KCTRLA |= (1 << 6); // Enable RTC 32kHz oscillator in standby
  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm); // Wait for RTC sync
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm; // Enable PIT with 1-second interval (32768 cycles at 32.768kHz)
  RTC.PITINTCTRL &= ~RTC_PI_bm; //Disable PIT interrupts
  // RTC.PITINTCTRL = RTC_PI_bm; // Enable PIT interrupt
  //RTC.PITCTRLA = 0; //Disable RTC interrupts
  //RTC.CTRLA = 0; //Disable RTC
  //TCB1.CTRLA &= ~(1 << TCB_ENABLE_bp);
 
  //Set all unused pins to poutput LOW to reduce power soncumption
  for(i=0; i<18; i++){
    if(i != 4 || i != 8 || i != 9 || i != 17){
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }

  //Initialize I2C
  pinMode(SDA, INPUT_PULLUP); //Needed to use internal pullups as I2C pullup
  pinMode(SCL, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(1e6);
  Wire.usePullups();

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
  tempData[0] = B01111010; //data to Dev_initial register, 11 max_line_num (default), set mode 1, 125kHz pwm_fre (default)
  tempData[1] = B00000100; //data to Dev_config1 register, 1us sw_blk (default), enable exponential scale dimming curve, phase shift off (default), cs_on_shift off (default)
  tempData[2] = B00000001; //data to Dev_config2 register, comp_group3/2/1 off (default), lod_removal disable (defualt), enable lsd_removal
  tempData[3] = B11110001; //data to Dev_config3 register, weak down deghost (default), vled-2v up deghost (default), 15mA maximum current (default), enable up deghost (default)
                           // Current: 000 = 7.5 mA, 001 = 12.5 mA, 010 = 25 mA, 011 = 37.5 mA, 100 = 50 mA, 101 = 75 mA, 110 = 100 mA
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, startRegAddress, i2cWrite, 4, &tempData[0]);

  pinMode(sensor, INPUT);
  for(i = 0; i < n_leds; i++) tempData[i] = 0x00;
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &tempData[i]); //Max I2C length isdot_onoff0
  for(i = 0; i < n_leds; i++) tempData[i] = 0xFF;
  for(i = 0; i < n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dc0+i, i2cWrite, 31, &tempData[i]);
  for(i=0; i<n_leds; i+=31) i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, pwm_bri0+i, i2cWrite, 31, &tempData[i]); //Max I2C length isdot_onoff0
  startComparator();
  tempData[0] = 0x00; //Disable chip
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, chip_en_reg, i2cWrite, 1, &tempData[0]);
}

void loop() {
  if(power_down) powerDown();
  else monitorIRStream();
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

//https://github.com/SpenceKonde/megaTinyCore/tree/master/megaavr/libraries/Comparator
//https://github.com/grughuhler/attiny/blob/main/attiny_ac/attiny_ac.ino
void startComparator(){
  sensor_baseline = 255;

  pinMode(output_pin, OUTPUT);
  Comparator.input_p = comparator::in_p::in1;       // pos input PA7.  See datasheet
  Comparator.input_n = comparator::in_n::dacref;    // neg pin to the DACREF voltage
  Comparator.reference = comparator::ref::vref_vdd; // Set the DACREF voltage
  Comparator.dacref = sensor_baseline;

  Comparator.hysteresis = comparator::hyst::medium;  // Use 50mV hysteresis
  Comparator.output = comparator::out::disable;      // Enable output PB3
  Comparator.output_initval = comparator::out::init_high; // Output pin high after initialization
  Comparator.attachInterrupt(ac_interrupt, CHANGE);
  AC0.CTRLA |= AC_RUNSTDBY_bm;  //Allow the comparator to run when the microcontroller is in idle
  AC0.CTRLA |= AC_LPMODE_bm; //Allow the comparator to run in low-power mode with the cost of a slower propagation
  AC0.CTRLA &= ~(AC_OUTEN_bm); //Disable output
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

void monitorIRStream(){
  uint16_t i, j;
  uint8_t nec_byte[16];
  bool nec_valid; //Whether the NEC encoding is correct
  bool blank_display; //Whether any pixels are on in the frame

  set_sleep_mode(SLEEP_MODE_STANDBY);  // Lower power than idle, retains AC
  sleep_enable();
  sleep_cpu(); // Device goes to sleep here, wakes on AC interrupt
  sleep_disable(); // Disable after wake (optional)
  tempData[0] = 0x01; //data to Chip_en register, enable chip
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, chip_en_reg, i2cWrite, 1, &tempData[0]); //Send enable command
  timeout_timer = 0;
  while((sensor_state != 1 || interrupt_duration < start_com_duration) && timeout_timer < wake_timeout); //Wait for LED to go low again
  if(interrupt_duration > power_down_duration && timeout_timer < wake_timeout){ //If an extended power-down pulse was received, power down device
    power_down = 1;
    return;
  }
  interrupt_duration = 0;
  timeout_timer = 0;
  while(!interrupt_duration && timeout_timer < comm_timeout); //Wait for first timing pulse
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
  blank_display = true;
  for(i=0; i<n_rows*2 && nec_valid; i+=2){ //Verify that NEC encoding is valid
    nec_byte[i+1] ^= nec_byte[i];
    if(nec_byte[i+1] != 0xFF) nec_valid = false;
    if(nec_byte[i]) blank_display = false;
  }
  if(i<n_rows*2 || j<8 || !nec_valid || blank_display){ //If an error happened during communication
    for(i=0; i<24; i++) display[i] = 0; //Zero out the display array
    display[1] |= 1; //Turn on red indicator LED 
    blank_display = true;
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
    display[1] |= 1; //Turn on red indicator LED 
  }
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
  if(blank_display){ //Disable chip if all LEDs are off
    tempData[0] = 0x00; //data to Chip_en register, enable chip
    i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, chip_en_reg, i2cWrite, 1, &tempData[0]);
    delayMicroseconds(200);
  }
  else vsync();
}

void powerDown(){
  uint8_t i;
  bool j;

  if(power_down == 1){ //Send indication that device is powering down
    AC0.CTRLA &= ~AC_RUNSTDBY_bm;  //Disable comparator in idle to save power
    delay(1);
    flashLED(2);
    power_down = 2;
    pit_counter = 0; //Reset the PIT counter to catch wake commands
    RTC.PITINTCTRL |= RTC_PI_bm; // Enable PIT interrupt
  }
  set_sleep_mode(SLEEP_MODE_STANDBY);  // Lower power than idle, retains AC
  sleep_enable();
  sleep_cpu(); // Device goes to sleep here, wakes on AC interrupt
  sleep_disable(); // Disable after wake (optional)
  if(!Comparator0.read()){ //Turn on the read LED if the comparator sees an IR pulse
    if(pit_counter++){ //If LED is on for more than a second, wake from sleep
      flashLED(1);
      AC0.CTRLA |= AC_RUNSTDBY_bm;  //Allow the comparator to run when the microcontroller is in idle
      power_down = 0;
      timeout_timer = 0;
      while(!Comparator0.read()){ //Wait for LED to turn off - waking at least 1x per second
        set_sleep_mode(SLEEP_MODE_STANDBY);  
        sleep_enable();
        sleep_cpu(); // Device goes to sleep here, wakes on AC interrupt
        sleep_disable(); // Disable after wake (optional)
        flashLED(1);
      }
      RTC.PITINTCTRL &= ~RTC_PI_bm; //Disable RTC interrupts
      flashLED(2);
    } 
  }
  else pit_counter = 0; //Otherwise reset the PIT counter
}

void flashLED(uint8_t n_flashes){
  uint8_t i;
  for(i=0; i<24; i++) display[i] = 0x00; //Zero out the display array
  display[1] |= 1; //Turn on red indicator LED 
  i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
  vsync();
  for(i=0; i<n_flashes; i++){ //Flast red LED to confirm sleep command receieved
      tempData[0] = 0x01; //data to Chip_en register, enable chip
      i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, chip_en_reg, i2cWrite, 1, &tempData[0]); //Send enable command
      delay(1);
      i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, dot_onoff0, i2cWrite, 24, &display[0]);
      vsync();
      delay(200);
      tempData[0] = 0x00; //data to Chip_en register, enable chip
      i2cSendReceive(I2C_TARGET_ADDRESS_INDEPENDENT, chip_en_reg, i2cWrite, 1, &tempData[0]); //Send disable command
      delay(200);
  }
}

//RTC.PITCTRLA = 0; //Disable RTC interrupts
//AC0.CTRLA |= AC_RUNSTDBY_bm;  //Allow the comparator to run when the microcontroller is in idle
// RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm; // Enable PIT with 1-second interval (32768 cycles at 32.768kHz)
// RTC.PITINTCTRL = RTC_PI_bm; // Enable PIT interrupt
// RTC.PITCTRLA = 0; //Disable RTC interrupts
// RTC.CTRLA = 0; //Disable RTC

void vsync(){
  PORTC.OUTSET = PIN0_bm;
  delayMicroseconds(10);
  PORTC.OUTCLR = PIN0_bm;
}