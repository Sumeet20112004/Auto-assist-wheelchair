// Pins for hall sensors
const int LEFT_HALLSENSOR_PIN = 2;
const int RIGHT_HALLSENSOR_PIN = 3;

// Time interval (in milliseconds) over which to count pulses
const int TIME = 100;  // 100ms 

// Volatile pulse counters (must be volatile because they are updated in interrupts)
volatile long LEFT_PULSE_COUNT = 0;
volatile long RIGHT_PULSE_COUNT = 0;

// Conversion factor to calculate RPM:
// RPM = (Pulses in TIME ms) * (60000 ms/min) / (TIME * pulses_per_revolution)
// Assuming pulses_per_revolution = 60 => (60000 / (TIME * 60)) = 10
// But your constant is hardcoded (30000 / TIME), make sure this matches your hardware.
const float TRANSITION_PER_REVOLUTION = 30000.0 / TIME;  // Adjust this based on encoder resolution

// Interrupt Service Routines (ISRs) for pulse counting
void LEFT_COUNTER() {
  LEFT_PULSE_COUNT++;
}

void RIGHT_COUNTER() {
  RIGHT_PULSE_COUNT++;
}

void setup() {
  Serial.begin(9600);  // Begin serial communication

  // Set up hall sensor pins as inputs
  pinMode(LEFT_HALLSENSOR_PIN, INPUT);
  pinMode(RIGHT_HALLSENSOR_PIN, INPUT);

  // Attach interrupts to count transitions on sensor state changes
  attachInterrupt(digitalPinToInterrupt(LEFT_HALLSENSOR_PIN), LEFT_COUNTER, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_HALLSENSOR_PIN), RIGHT_COUNTER, CHANGE);
}

void loop() {
  // Reset counters safely (disable interrupts temporarily)
  noInterrupts();
  LEFT_PULSE_COUNT = 0;
  RIGHT_PULSE_COUNT = 0;
  interrupts();

  // Wait for TIME ms
  delay(TIME);  

  // Copy the pulse counts again with interrupts disabled
  noInterrupts();
  long LEFT_PULSE = LEFT_PULSE_COUNT;
  long RIGHT_PULSE = RIGHT_PULSE_COUNT;
  interrupts();

  // Convert pulse count to RPM (use float for precision)
  int LEFT_RPM = (int)(((float)LEFT_PULSE) * TRANSITION_PER_REVOLUTION);
  int RIGHT_RPM = (int)(((float)RIGHT_PULSE) * TRANSITION_PER_REVOLUTION);

  // Format and send over serial in the form "LLLL|RRRR" (padded to 4 digits)
  char DATA_SENT[12];
  sprintf(DATA_SENT, "%04d|%04d", LEFT_RPM, RIGHT_RPM);
  Serial.println(DATA_SENT);
}
