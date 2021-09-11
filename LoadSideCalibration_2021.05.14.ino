/**************************************************************************************
 * Load Controller Calibration Program
 *  - Use this code to program the Load Side Arduino (Uno) to read data from turbine.
 *  - Accepts Commands from a software serial port to change load setting values,
 *    command +12V on/off, and communicate servo brake commands to the turbine.
 *  - MUST BE PAIRED WITH TURBINE SIDE CALIBRATION PROGRAM!!
 **************************************************************************************/

// Libraries Required
#include <Wire.h>;              // For I2C communication with Analog to Digital Converter
#include <Adafruit_MCP4725.h>;  // Analog to Digital Converter for Load Control
#include <SoftwareSerial.h>;    // Software Serial Communications library
//#include <math.h>;              // Math Libray for calculations (Not needed for calibration)

Adafruit_MCP4725 dac;           // Set up instance of the Digital to Analog Converter
SoftwareSerial SoftSerial(3, 5);  // Set up Software serial for RX in on Pin 3, TX out on Pin 5

/***************************************************************************************
 * Global Constants and Variables
 * - This section defines variables used throughout the program
 * *************************************************************************************/
// Arduino Input/Output Pin Mapping
const int iLoadVoltage_Pin = A1;    // Analog input pin for measuring load voltage (0-12.5V)
const int iLoadCurrent_Pin = A0;    // Analog input pin for measuring load current (-1 to +4A)
const int iLoadPower_Pin = 6;       // Digital output pin for commanding P-MOS for +12V to turbine

// Timing Variables
unsigned long ulStartTime;            // Timing Variable
unsigned long ulEndTime;              // Timing Variable
int iElapsedTime;                     // Timing Variable
// Brake Servo Motor Enable
byte bServoEnabled = 0;               // Servo Enabled Boolean Variable
// Load Power Enable Variable
byte bLoad12VEnabled = 0;                  // Load Powered Boolean Variable
// Load and Brake Setting (as global variables)
int iLoadSetting = 0;                 // Load Setting
int iBrakeSetting = 1000;             // Brake Setting
// E-stop variable
byte bEStop_Now = 1;                  // Current e-stop button condition
byte bEStop_Last = 1;                 // Last cylce e-stop button condition
// Mode Variable
char cMode = 'A';

/* Setup Function */
void setup() {
  // put your setup code here, to run once:
  // Set the pin functions
  pinMode(iLoadVoltage_Pin, INPUT);
  pinMode(iLoadCurrent_Pin, INPUT);
  pinMode(iLoadPower_Pin, OUTPUT);
  // Make sure that the Load Power (+12V) is disabled
  digitalWrite(iLoadPower_Pin, LOW);
  // Start the Serial port to transmit results to Serial Monitor. Use speed 38400 bps.
  Serial.begin(38400);
  // Start the Software Serial port to recieve commands. Use speed 38400 bps.
  SoftSerial.begin(38400);
  ulStartTime = millis();
  //Setting up the load (testing purposes)
  dac.begin(0x62);
  dac.setVoltage(0, false);
}

/*****************************************************************************
/* Main Program Loop 
/*****************************************************************************/
void loop() {
  // Loop Constants
  const char cStartChar = '<';          // Serial Input Start Character
  const int iRXTimeOut = 100;           // Serial Input Timeout
  const int iRXSize = 34;               // Serial Input Length (Characters from Turbine)
  const int iTXSize = 56;               // Serial Output (To Turbine) Length
  
  // Varaibles used throughout the loop
  int iGenVoltage = 0;                  // Generator voltage
  int iGenCurrent = 0;                  // Generator current
  int iGenPower = 0;                    // Generator Power
  int iOutputVoltage = 0;               // Turbine Output Voltage
  int iOutputCurrent = 0;               // Turbine Output Current
  int iOutputPower = 0;                 // Turbine Output Power
  float fLoadVoltage = 0;               // Load voltage (float)
  int iLoadVoltage = 0;                 // Load Voltage (integer)
  float fLoadCurrent = 0;               // Load current (float)
  int iLoadCurrent = 0;                 // Load Current (integer)
  float fLoadPower = 0;                 // Load power (float)
  int iLoadPower = 0;                   // Load Power (integer)
  int iRPM = 0;                         // RPM
  int i=0;                              // Loop counting variable
  float fRPMSetPoint = 0;
  int iRPMSetPoint = 0;
  float fDRPM = 0;
  int iDRPM = 0;
  float fDLoadSetting = 0;
  int iDLoadSetting = 0;

  // Hardware Serial Communication Variables
  char cChar;
  unsigned long lRXStartTime = 0;
  unsigned int iRXElapsedTime = 0;
  bool bRXTimeout = false;
  bool bRXComplete = false;
  bool bStartCharFound = false;
  char cString[iTXSize];                // String to transmit data

  //Software Serial Communication Variables
  char cCommand = 0;                    // Serial Port Command Character
  int iCommand = 0;                     // Serial Port Command Integer
 
 /********************************************************************************
  /* RECIEVE DATA FROM TURBINE
  /********************************************************************************/ 
  lRXStartTime = millis(); // Start time of waiting for data recieve
  // While the data frame has NOT been completed AND as long
  // as there has not been a timeout...
  while(!bRXTimeout && !bRXComplete) {
    // Determine if the start character has been found in the serial RX buffer
    if(Serial.available() && !bStartCharFound) {
      cChar = Serial.read();    // Read the next character in the buffer
      if(cChar == cStartChar) { 
        bStartCharFound = true; // If the start character is found, set the flag to true
      }
    }
    // If the start character is found AND the right number of characters are in the 
    // RX buffer, parse the received data.
    if(bStartCharFound && (Serial.available() >= iRXSize-2)) {
      cMode = Serial.read();                // Mode Character
      iRPM = Serial.parseInt();             // RPM
      iGenVoltage = Serial.parseInt();      // Genenerator Voltage
      iGenPower = Serial.parseInt();        // Generator Power
      iOutputVoltage = Serial.parseInt();   // Turbine Output Voltage
      iOutputCurrent = Serial.parseInt();   // Turbine Output Current
      bEStop_Now = Serial.parseInt();      // Last E-stop Value    
      bRXComplete = true;
    }

    // Compute elapsed time and compare to timeout
    iRXElapsedTime = millis()-lRXStartTime;
    if(iRXElapsedTime >= iRXTimeOut) { 
      bRXTimeout = true; // If timeout has been exceeded, exit
    } 
  }
 
  /**************************************************************** 
  /* LOAD SIDE MEASUREMENTS
  /****************************************************************/
  // Accumulate multiple readings. The number of accumulated readings
  // must be considerd in the conversion/calibration factors below.
  // (which are currently assuming 4 samples are accumulated)
  while(i<4) {
    fLoadVoltage = fLoadVoltage + analogRead(iLoadVoltage_Pin);
    fLoadCurrent = fLoadCurrent + analogRead(iLoadCurrent_Pin);
    i++;
  }
  
  // Use i-v sensor calibration data to convert the analog input values into scaled
  // currents and voltages. Use floating point math, then cast as integers for transmission.
  fLoadVoltage = 3.09563*fLoadVoltage + 33.14421; // from 0-12V_A Calibration
  fLoadCurrent = 1.22409*fLoadCurrent - 978.136; // from 0-12V_A Calibration
  // Here, both voltage and current are on the 10E-3 scale, giving microwatts as the result.
  // Divide by 10,000 to scale to centiwatts again.
  fLoadPower = fLoadVoltage * fLoadCurrent / 10000.0;
  // Cast each floating point value as an integer for transmission over serial.
  iLoadVoltage = (int) fLoadVoltage;
  iLoadCurrent = (int) fLoadCurrent;
  iLoadPower = (int) fLoadPower;

  /**************************************************************** 
  /* Recieve Software Serial Commands
  /****************************************************************/
  // See if there has been a SOFTWARE serial command received
  // Use for controlling Load command and Brake Command via Serial interface.
  // See if the serial port has more than 2 characters of data available in the serial buffer
  if(SoftSerial.available()>2) {
    cCommand = SoftSerial.read(); // Read the first character in the buffer
    iCommand = SoftSerial.parseInt(); // Read the remaining characters as an integer (ignores anything else)
    // If the starting character was a capital L, then change the load setting
    if(cCommand == 'L') {
      // Check first to make sure that the +12V supply is not enabled!
      if(bLoad12VEnabled == 0) {
        // If +12V is not enabled, then go ahead and change the load setting.
        iLoadSetting = iCommand; 
        SoftSerial.print("Load = "); SoftSerial.println(iLoadSetting);
        dac.setVoltage(iLoadSetting, false); 
      } else {
        // If +12V is enabled, then prevent the user from changing the load setting.
        iLoadSetting = 0;
        dac.setVoltage(iLoadSetting, false);
        SoftSerial.println("+12V Enabled, Load = 0");
      }
    }
    // If the Starting Character was a capital V, then change the state of the +12V load
    // Remember to set the DAC to zero to prevent shoot-through current!!
    if(cCommand == 'V') {
      if(iCommand == 0) {
        bLoad12VEnabled = 0;
        digitalWrite(iLoadPower_Pin, LOW);
        SoftSerial.println("+12V OFF");
      } else {
        bLoad12VEnabled = 1;
        iLoadSetting = 0;                   // SET THE LOAD TO ZERO!!
        dac.setVoltage(iLoadSetting, false);
        delay(10);                          // Wait for the load to settle.
        digitalWrite(iLoadPower_Pin, HIGH); // Then turn on the +12V power
        SoftSerial.println("+12V ON");
      } 
    }
    // If the starting character was a capital B, change the brake command setting
    if(cCommand == 'B') {
      iBrakeSetting =  iCommand; // Read the remaining characters as an integer (ignores anything else)
      // Error Trapping - prevent too large or small of command value to brake servo
      if(iBrakeSetting < 1000) { iBrakeSetting = 1000;}
      if(iBrakeSetting > 2000) { iBrakeSetting = 2000;}
      SoftSerial.print("Brake = "); SoftSerial.println(iBrakeSetting);
    }
    // Servo Motor Enable
    if(cCommand == 'M') {
      if(iCommand == 0) {
        bServoEnabled = 0;
      } else {
        bServoEnabled = 1;
      }
      SoftSerial.print("ServoEn = "); SoftSerial.println(bServoEnabled);
    }
    //Flush the remaining characters to empty the serial buffer (endlines and carriage returns)
    while(SoftSerial.available() > 0) {
      cCommand = SoftSerial.read();
    }
  }

  /****************************************************************************** 
  /* Transmit Data to Serial Port and Turbine - Monitor with Serial Monitor
  /******************************************************************************/
  // Measure the amount of time that has elapsed (in milliseconds);
  // Loop time will depend upon how fast the rotor is spinning (due to the pulseIn function)
  ulEndTime = millis();
  iElapsedTime = ulEndTime - ulStartTime;
  ulStartTime = ulEndTime; // Reset the start time for the next loop
  // Send measurements to Serial port, comma separated
  // Format the string to include:
  // - Start Character ("<")
  // - Mode Character
  // - Elapsed Loop Time
  // - RPM (deci-RPM)
  // - Generator Voltage (cV)
  // - Generator Power (cW)
  // - Load Voltage (mV)
  // - Load Current (mV)
  // - Load Power (cW)
  // - Load Command Setting
  // - Brake Command Setting
  // - Servo Enable Flag (0 = OFF, 1 = Enabled)
  // - Estop (1 = NO Emergency, 0 = ESTOP)
  // - +12V Supply Enabled (0 = OFF, 1 = ON)
  // Zero Pad values to specified fixed length
  sprintf(cString,"<%c,%03d,%05d,%04d,%04d,%05d,%05d,%04d,%04d,%04d,%01d,%01d,%01d\n", 
    cMode, iElapsedTime, iRPM, iGenVoltage, iGenPower, iLoadVoltage, iLoadCurrent, 
    iLoadPower, iLoadSetting, iBrakeSetting, bServoEnabled, bEStop_Now, bLoad12VEnabled);
  Serial.print(cString);
  // Wait for the serial buffer to complete sending before looping again
  // So the program doesn't run too fast and overflow the serial buffer
  // Based on a 38400 bps baud rate, this should take less than 10ms, giving a loop
  // frequency around 100 Hz. But, this is limted by the rotational speed of the rotor
  // due to the wait for the pulseIn function.
  // Note: This may not be necessary... we can see from the timing if it is.
  Serial.flush();
  // Additional wait time (adjust as necessary if moving to quickly...)
  // Reduce (or remove) wait time (in ms) to capture faster data for processing in Excel/Matlab
  delay(50);
}
