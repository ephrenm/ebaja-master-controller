// eBaja Master Controller v6 - modified 10-9-23
// Changelog:
// Added SDC sensing and pre-charge sequence, shifted some pins around for testing
// Added RTD button which is active low with internal pullup resistor to prevent issues with floating input pins
// Added pre-charge button for manual pre-charging
// Changed brake signal to active low, also using internal pull-up resistor on that digital pin
// [TODO]
// Add brake switch input to enter RTD mode
// Add RTD enable signal output to send to ETC Arduino
// analogWrites for MOSFET switching (we wanna vary duty cycle of CF and WP based on temperature, but no temp sensors for Phase I)
// Determine whether pressing RTD button twice should turn the car off
// [Phase II]
// Add KTY84 temp sensor reading w/ resistor divider to get motor temp - must calibrate our readings
// Add UART input to read temps from generic BMS
// Add CAN shield for reading cell temps from MUX
// Add GLV voltage sensing using resistor-divider to analog input. Make the MEGA do something when GLV battery is low

// Operating States
// 0 - everything is off, only way to turn things on is to connect a computer over USB - **think about HV isolation**
// 1 - GLVMS has been enabled - 12V turns on Arduino and everything directly powered by the fusebox is able to turn on
//     Now we can do things like check throttle/brake position, enable cooling fans, water pump, brake light
// 2 - TSMS enabled - supply for the AIRs & pre-charge circuit is completed - only now can the pre-charge sequence occur
// 3 - Pre-charge Button pressed: added a manual precharge button so that pre-charging doesn't occur automatically when TSMS is inserted
//     Additionally, checking SDC status pin when pre-charge button is pressed to ensure that SDC is completed when TSMS is inserted
// 4 - RTD Mode entered - only possible when no major faults present, pre-charge is complete, brake pedal depressed, and RTD button pressed
//     Send enable signal to ETC Arduino using a digital pin so that it can output a throttle signal to I2C
// With all these different operating states, when can we/can we not connect a computer to motor controller, main Arduino, ETC Arduino?
// Throughout this process, think about how everything can fail and what logic the car can use to ensure everything is working

// Error checks
// Ground reference error check: is SDC pin going crazy?
// Pre-charge/AIR checks: how to tell if AIRs fail to turn on? (test bed issue) -> use auxiliary contacts?
//    As a result: voltage during pre-charge will spike up instead of curving like a normal pre-charge

// Note about pin defintions: the green MEGA shield numbers the terminal blocks, but not the black pin headers. Check a normal MEGA to see black header pin numbers

// Defining relay pins
int AIRplusRelay = 15;
int AIRminusRelay = 14;
int prechargeRelay = 16;
int dischargeRelay = 17;
int RTDspeakerRelay = 27;
int eLockRelay = 29;

// Defining MOSFET pins
int waterpumpMOS = 0;
int coolingfansMOS = 0;
int brakelightMOS = 0;

// Shutdown circuit status pin, status variable, and counter
int SDCpin = 26;
int SDCreading = 0;
bool SDCstate = false;

// Variable to store precharge state (always initialize as false to assume precharge did not happen)
int prechargeButton = 11;
int prechargeButtonRead = 0;
bool prechargeState = false;

// Variables to manage RTD button & state
int rtdButton = 12;
int rtdButtonRead = 0;
bool RTDstate = false;

// Other I/O Pins
int busVoltage = 0;   // Analog pin which will read bus voltage (advanced feature, not likely to be implemented on eBaja)
int brakeSignal = 0;  // Digital pin which will be LOW when brake pedal is pressed (signal comes from brake switch)
int RTDout = 0;       // Digital pin which will be HIGH once RTD mode enabled (goes to ETC Arduino)

void setup() {
  Serial.begin(9600);

  // Setting previously chosen pins as inputs/outputs
  pinMode(dischargeRelay, OUTPUT);
  pinMode(prechargeRelay, OUTPUT);
  pinMode(AIRminusRelay, OUTPUT);
  pinMode(AIRplusRelay, OUTPUT);
  pinMode(RTDspeakerRelay, OUTPUT);
  pinMode(eLockRelay, OUTPUT);
  pinMode(waterpumpMOS, OUTPUT);
  pinMode(coolingfansMOS, OUTPUT);
  pinMode(brakelightMOS, OUTPUT);
  pinMode(rtdButton, INPUT_PULLUP);
  pinMode(brakeSignal, INPUT);
  pinMode(SDCpin, INPUT);
  pinMode(prechargeButton, INPUT_PULLUP);

  // Setting all control relays to HIGH when Arduino is initialized to turn them off
  digitalWrite(AIRplusRelay, HIGH);
  digitalWrite(AIRminusRelay, HIGH);
  digitalWrite(prechargeRelay, HIGH);
  digitalWrite(dischargeRelay, HIGH);
  digitalWrite(eLockRelay, HIGH);
  digitalWrite(RTDspeakerRelay, HIGH);

  // Beep the RTD speaker to signify that setup is complete
  digitalWrite(RTDspeakerRelay, LOW);
  delay(3);
  digitalWrite(RTDspeakerRelay, HIGH);
}

void loop() {
  // Checking digital pin input continuously to see if SDC is properly functioning
  // The moment its voltage drops, the SDC opens and the car shuts off
  // Resistor divided w/ three 1k resistors after HVD Interlock. Check for proper ground reference!
  SDCreading = digitalRead(SDCpin);
  if (SDCreading == HIGH) {
    SDCstate = true;
  } else {
    SDCstate = false;
  }

  // If the SDC opens, then open all relays and reset precharge and ready-to-drive states
  if (SDCstate == false) {
    // Turning off ALL relays if SDC is broken
    digitalWrite(AIRplusRelay, HIGH);
    digitalWrite(AIRminusRelay, HIGH);
    digitalWrite(prechargeRelay, HIGH);
    digitalWrite(dischargeRelay, HIGH);
    digitalWrite(eLockRelay, HIGH);
    digitalWrite(RTDspeakerRelay, HIGH);
    prechargeState = false;
    RTDstate = false;  // Also setting RTDstate back to false since we exit RTD state whenever SDC opens
  }

  prechargeButtonRead = digitalRead(prechargeButton);
  if (prechargeButtonRead == 0) {
    if (!prechargeState && SDCstate == true) {  // If statement so that pre-charge only runs once
      Serial.println("Precharge sequence starting");
      digitalWrite(dischargeRelay, LOW);  // discharge relay is NC, so must give it power by commanding control relays LOW to disconnect outputs
      digitalWrite(AIRminusRelay, LOW);
      digitalWrite(prechargeRelay, LOW);
      delay(3000);  // insert appropriate delay here for pre-charge to complete
      digitalWrite(prechargeRelay, HIGH);
      digitalWrite(AIRplusRelay, LOW);
      prechargeState = true;
      Serial.println("Precharge sequence ended");
    }
  }

  // Watch for RTD switch input, then check if SDC is good (again), pre-charge has completed successfully, and brake pedal is being pressed
  // Also send RTd output to ETC Arduino on a digital pin
  rtdButtonRead = digitalRead(rtdButton);
  if (rtdButtonRead == LOW) {
    if (SDCstate == true && prechargeState == true) {  // Add check for pressing brake pedal
      if (RTDstate == false) {
        digitalWrite(eLockRelay, LOW);
        Serial.println("RTD Button Pressed");

        // Pin to send RTD signal to ETC Arduino
        digitalWrite(RTDout, HIGH);

        // Writing RTD speaker LOW for 2s to turn on, then writing HIGH
        digitalWrite(RTDspeakerRelay, LOW);
        delay(2000);
        digitalWrite(RTDspeakerRelay, HIGH);

        // Setting RTDstate to 1 so that we only "turn on" the car once
        RTDstate = true;
      }
    }
  }

  // Steal motor temp info from the built-in KTY84 thermistor in the motor
  // Use KTY84 w/ resistor divider - will have to calibrate the sensor and compare to motor controller values
  // Once we have MOSFETs, use motor temp to determine duty cycle to run MOSFETs

  // Enable brake light whenever brake light signal input is LOW - this code is basically done
  if (brakeSignal == LOW) {
    digitalWrite(brakelightMOS, HIGH);
  } else {
    digitalWrite(brakelightMOS, LOW);
  }
}