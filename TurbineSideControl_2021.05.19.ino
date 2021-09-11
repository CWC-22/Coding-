/**************************************************************************************
 * Turbine Side Calibration Program
 *  - Use this code paired with the Load Side Calibration Program to read data from
 *    and Control the turbine for calibration purposes ONLY
 *  - THIS CODE DOES NOT CONTAIN Turbine Control Elements - it only reads and transmits
 *    data to the load!!
 * *************************************************************************************/

// Libraries Required
#include <Servo.h>;
Servo myservo;

/***************************************************************************************
 * Global Constants and Variables
 * - This section defines variables used throughout the program
 * *************************************************************************************/

// Arduino Input/Output Pin Mapping
const int iGenVoltage_Pin = A1;     // Analog input pin for Generator voltage (0-60V)
const int iGenCurrent_Pin = A0;     // Analog input pin for Generator current (0-1A)
const int iOutputVoltage_Pin = A3;  // Analog input pin for Output voltage (0-12.5V)
const int iOutputCurrent_Pin = A2;  // Analog input pin for Output current (-1 - 4A)
const int iRPM_Pin = 7;             // Digital pin for measuring encoder pulses
const int iEStop_Pin = 4;           // Digital pin to monitor e-stop button
const int iBrake_Pin = 10;          // Brake Servo Control pin
const int iServoEnable_Pin = 6;     // Brake Servo N-FET Power Enable Pin

// Load Setting Variables 
int iLoadSetting = 0;               // Load Setpoint Value
int iLoad12VEnabled = 0;            // Is +12V Load Power enabled?

// Brake Variables
int iBrakeSetting = 1600;           // Brake Setting
const int iBrakeMax = 2000;         // Maximum brake setting value (full closed)
const int iBrakeMin = 1550;         // Minimum brake setting value (when open)
int iServoEnabled = 0;              // Servo Power Enabled Boolean Variable

// E-stop variable
int iEStop_Now = 1;                  // Current e-stop button condition
int iEStop_Last = 1;                 // Last cylce e-stop button condition

// Mode Variable
char cMode = 'A';

/* Setup Function */
void setup() {
  // put your setup code here, to run once:
  // Set the pin states
  pinMode(iGenVoltage_Pin, INPUT);
  pinMode(iGenCurrent_Pin, INPUT);
  pinMode(iOutputVoltage_Pin, INPUT);
  pinMode(iOutputCurrent_Pin, INPUT);
  pinMode(iRPM_Pin, INPUT);
  pinMode(iServoEnable_Pin, OUTPUT);
  // Start the Serial port to transmit results to Load. Use speed 38400 bps.
  Serial.begin(38400);  

  myservo.attach(iBrake_Pin);         // attaches the servo object to the correct pin
}

/*****************************************************************************
/* Main Program Loop 
/*****************************************************************************/
void loop() {
  // Loop Constants
  const char cStartChar = '<';          // Serial Input Start Character
  const int iRXTimeOut = 200;           // Serial Input Timeout
  const int iRXSize = 56;               // Serial Input Length (Characters from Load)
  const int iTXSize = 33;               // Serial Output (To Load) Length
  
  // Varaibles used throughout the loop
  float fGenVoltage = 0;                // Generator voltage
  int iGenVoltage = 0;
  float fGenCurrent = 0;                // Generator current
  int iGenCurrent = 0;
  float fGenPower = 0;                  // Generator power
  int iGenPower = 0;
  float fOutputVoltage = 0;             // Output Voltage
  int iOutputVoltage = 0;
  float fOutputCurrent = 0;             // Output Current
  int iOutputCurrent = 0;
  float fOutputPower = 0;               // Output Power
  int iOutputPower = 0;
  int iLoadVoltage = 0;                 // Load Voltage
  int iLoadCurrent = 0;                 // Load Current
  int iLoadPower = 0;                   // Load Power
  
  unsigned long ulRPM_Pulse_Time = 0;   // Encoder Pulse Time (in us)
  int iRPM = 0;                         // Calculated RPM
  int i=0;                              // Loop counting variable

  // Serial Communication Variables
  char cChar;
  unsigned long lRXStartTime = 0;
  unsigned int iRXElapsedTime = 0;
  bool bRXTimeout = false;
  bool bRXComplete = false;
  bool bStartCharFound = false;
  int iIgnore = 0;                      // Integer variable to accept values to ignore

  char cString[iTXSize];                // String to transmit data

  /********************************************************************************
  /* RECIEVE DATA FROM LOAD
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
    if(bStartCharFound && Serial.available() >= iRXSize-1) {
      cMode = Serial.read();                // Mode Character
      iIgnore = Serial.parseInt();          // Elapsed Time
      iRPM = Serial.parseInt();             // Last Reported RPM
      iGenVoltage = Serial.parseInt();      // Last Reported Genenerator Voltage
      iGenPower = Serial.parseInt();        // Last Reported Generator Power
      iLoadVoltage = Serial.parseInt();     // Last Reported Load Voltage
      iLoadCurrent = Serial.parseInt();     // Last Reported Load Current
      iLoadPower = Serial.parseInt();       // Last Reported Load Power
      iLoadSetting = Serial.parseInt();     // Last Reported Load Setting
      iBrakeSetting = Serial.parseInt();    // Last Reported Brake Setting
      iServoEnabled = Serial.parseInt();    // Is the Servo Enabled?
      iEStop_Now = Serial.parseInt();       // Last E-stop Value
      iLoad12VEnabled = Serial.parseInt();  // Is the load +12V power enabled?      
      bRXComplete = true;
    }

    // Compute elapsed time and compare to timeout
    iRXElapsedTime = millis()-lRXStartTime;
    if(iRXElapsedTime >= iRXTimeOut) { 
      bRXTimeout = true; // If timeout has been exceeded, exit
    } 
  }
  
  /**************************************************************** 
  /* TURBINE MEASUREMENTS
  /****************************************************************/
  // Accumulate multiple readings. The number of accumulated readings
  // must be considerd in the conversion/calibration factors below.
  // (which are currently assuming 4 samples are accumulated)
  while(i<4) {
    // Using a pulseIn timeout of 10000us
    // gives a minimum measureable speed of about 30 RPM (0.5 RPS). this also prevents
    // the program from getting "stuck" waiting for a pulse when the rotor is not spinning.
    ulRPM_Pulse_Time = ulRPM_Pulse_Time + pulseIn(iRPM_Pin,HIGH,10000);
    // Between each pulse measurement, sample the voltages and currents
    fGenVoltage = fGenVoltage + analogRead(iGenVoltage_Pin);
    fGenCurrent = fGenCurrent + analogRead(iGenCurrent_Pin);
    fOutputVoltage = fOutputVoltage + analogRead(iOutputVoltage_Pin);
    fOutputCurrent = fOutputCurrent + analogRead(iOutputCurrent_Pin);
    i++;
  }

  // Now, check to see if the averaged RPM pulse time is less than the minimum, so
  // that there is not a divide by zero or small number error.
  if(ulRPM_Pulse_Time < 10) {
    // If the average pulse time is below the threshold, report back 0 RPM
    iRPM = 0;
  }
  else {
    // If the average pulse time is within the acceptable range, calculate the RPM.
    // Scale so that the result fits within an unsigned integer (Max of 32,768). Resulting
    // integer will represent tenths of RPM (when RPM = 1200, iRPM = 12000). Maximum
    // RPM reading is 3276 RPM - shouldn't need to go this fast!!
    iRPM = 12000000/ulRPM_Pulse_Time;
  }

  // Use i-v sensor calibration data to convert the analog input values into scaled
  // currents and voltages. Use floating point math, then cast as integers for transmission.
  // Don't forget to account for the number of samples accumulated above! (currently 4)
  // The Generator voltage maximum expressed in mV (60000 mV) will overflow an integer
  // (max of 32,768), so scale to centivolts (cV) instead.
  // NOTE: Why is this important to cast as an integer? In this case, not really...
  // BUT, serial communcation between Arduinos (i.e., between turbine and load Arduinos)
  // will be much easier to handle if the transmitted info is in integer form, not in float
  fGenVoltage = 1.485779*fGenVoltage + 17.94266; // from IV 0-60V Calibration
  fGenCurrent = 0.246214*fGenCurrent + 3.155505; // from IV 0-60V Calibration
  // Calculating Generator Power will give (10E-2 V * 10E-3 I) 10E-5 W. Divide by 1,000 to
  // scale the result as centiwatts (cW).
  fGenPower = fGenVoltage * fGenCurrent / 1000.0;
  fOutputVoltage = 3.09563*fOutputVoltage + 33.14421; // from 0-12V_A Calibration
  fOutputCurrent = 1.22409*fOutputCurrent - 978.136; // from 0-12V_A Calibration
  // Here, both voltage and current are on the 10E-3 scale, giving microwatts as the result.
  // Divide by 10,000 to scale to centiwatts again.
  fOutputPower = fOutputVoltage * fOutputCurrent / 10000.0;

  // Cast each floating point value as an integer for transmission over serial.
  iGenVoltage = (int) fGenVoltage;
  iGenCurrent = (int) fGenCurrent;
  iGenPower = (int) fGenPower;
  iOutputVoltage = (int) fOutputVoltage;
  iOutputCurrent = (int) fOutputCurrent;
  iOutputPower = (int) fOutputPower;

  /****************************************************************************
  /* Update Values based on data received from the Load
  /****************************************************************************/
  // Check on the E-STOP State
  iEStop_Last = iEStop_Now;             // Store the last button state
  if(digitalRead(iEStop_Pin) == HIGH) {
    iEStop_Now = 1;
  } else {
    iEStop_Now = 0;
  }
  //Do something if the E-stop changes state or there is a disconnect detected
  if((iEStop_Now < iEStop_Last) || ((iOutputVoltage - iLoadVoltage) > 5000)) {
    // If the state changes from 1 to 0, activate emergency stop protocol
    iEStop_Now = 0; // In case this results from disconnect detection
    iBrakeSetting = iBrakeMax;
    myservo.writeMicroseconds(iBrakeMax);    //change whatever number is in () to move actuator
    iServoEnabled = 1;
    digitalWrite(iServoEnable_Pin, HIGH);
  } else {
    // If not in emergency stop situation, write brake setting and enable/disable
    // the servo as indicated from the received data from the Load
    myservo.writeMicroseconds(iBrakeSetting); // Write the Brake Setting
    if(iServoEnabled == 1) {                  // Write the Servo Enabled Setting
      digitalWrite(iServoEnable_Pin, HIGH); 
    } else {
      digitalWrite(iServoEnable_Pin, LOW);
    }
  }
  
  /****************************************************************************
  /* Transmit data back to load
  /****************************************************************************/
  // Send measurements to Serial port, comma separated
  // Format the string to include:
  // - Starting Character ("<")
  // - Mode Character
  // - RPM (deci-RPM)
  // - Generator Voltage (cV)
  // - Generator Power (cW)
  // - Output Voltage (mV)
  // - Output Current (mA)
  // - E-Stop State (1 = No Emergency, 0 = ESTOP)
  // All values are zero padded to specified length
  sprintf(cString,"<%c,%05d,%04d,%04d,%05d,%05d,%01d",cMode, iRPM,iGenVoltage,iGenPower,iOutputVoltage, 
    iOutputCurrent,iEStop_Now);
  Serial.println(cString);
  // Wait for the serial buffer to complete sending before looping again
  // So the program doesn't run too fast and overflow the serial buffer
  // Based on a 38400 bps baud rate, this should take less than 10ms, giving a loop
  // frequency around 100 Hz. But, this is limted by the rotational speed of the rotor
  // due to the wait for the pulseIn function.
  // Note: This may not be necessary... we can see from the timing if it is.
  Serial.flush();
  // Additional wait time (adjust as necessary if moving to quickly...)
  // Reduce (or remove) wait time (in ms) to capture faster data for processing in Excel/Matlab
  //delay(1); 
}
