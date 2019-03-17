#include<Servo.h>

// Defines whether there is verbose logging on the Serial Monitor
// commented out = production
// not commented out = debug mode
#define VERBOSE

#define PERCENT_TOP 100
#define PERCENT_LOW 0
// MUST NOT be too less, otherwise we will get weird values
#define PULSE_IN_TIMEOUT 25000
// time a tick should roughly take.
#define ROUGH_TICK_TIME 1000 // 225

// It is important that this port is not connected to anything at all!
#define RANDOM_SEED_PORT 12

#define ROTOR_1_PORT 9
#define ROTOR_2_PORT 10
#define ROTOR_3_PORT 11
#define ROTOR_4_PORT 12

#define CHANNEL_1_PORT 5
#define CHANNEL_2_PORT 8
#define CHANNEL_3_PORT 7
#define CHANNEL_4_PORT 6

#define ROTOR_1_MIN_STRENGTH 900 // (60)
#define ROTOR_1_MAX_STRENGTH 2756

#define ROTOR_2_MIN_STRENGTH 990 // (60)
#define ROTOR_2_MAX_STRENGTH 2846

#define ROTOR_3_MIN_STRENGTH 900 // (60)
#define ROTOR_3_MAX_STRENGTH 2756

#define ROTOR_4_MIN_STRENGTH 900 // (60)
#define ROTOR_4_MAX_STRENGTH 2756

struct ValueSet
{
  int thrust;
  int movement_f_b;
  int movement_l_r;
  int rotation_l_r;
};

unsigned long start_time;

int last_thrust_value = 0;

int const INPUT_PORTS[] = {RANDOM_SEED_PORT, CHANNEL_1_PORT, CHANNEL_2_PORT, CHANNEL_3_PORT, CHANNEL_4_PORT};
// in the arduino world, there is no reliable way of knowing how long an array is, thus computing this at compile time
#define kInputPortLength (sizeof(INPUT_PORTS)/sizeof(int))

int const OUTPUT_PORTS[] = {ROTOR_1_PORT, ROTOR_2_PORT, ROTOR_3_PORT, ROTOR_4_PORT};
// in the arduino world, there is no reliable way of knowing how long an array is, thus computing this at compile time
#define kOutputPortLength (sizeof(OUTPUT_PORTS)/sizeof(int))


bool is_shutdown = false;

Servo rotor_1, rotor_2, rotor_3, rotor_4;

// This is the entry point for our program.
// This method is being called by the bootloader after its usual boot-up stuff
// and our chance to initalize some basic stuff(like input ports, rotors…)
void setup() {
  // should be close (if not equal) to zero, otherwise the bootloader is doing quite crazy stuff?
  start_time = millis();

  // Initalize input pins for reading values from them
  for (int i = 0; i < kInputPortLength; i++) {
    pinMode(INPUT_PORTS[i], INPUT);
  }

  // Initalize output pins for sending values to them
  for (int i = 0; i < kOutputPortLength; i++) {
    pinMode(OUTPUT_PORTS[i], OUTPUT);
  }

  // seed the (pseudo-)random number generator asap on an **unconnected** pin
  randomSeed(analogRead(RANDOM_SEED_PORT));

  // Connect to a (potential) computer for outputting debug information on baud 9600
  Serial.begin(9600);

  // Set output pins
  rotor_1.attach(ROTOR_1_PORT, ROTOR_1_MIN_STRENGTH, ROTOR_1_MAX_STRENGTH);
  rotor_2.attach(ROTOR_2_PORT, ROTOR_2_MIN_STRENGTH, ROTOR_2_MAX_STRENGTH);
  rotor_3.attach(ROTOR_3_PORT, ROTOR_3_MIN_STRENGTH, ROTOR_3_MAX_STRENGTH);
  rotor_4.attach(ROTOR_4_PORT, ROTOR_4_MIN_STRENGTH, ROTOR_4_MAX_STRENGTH);

  Serial.print("Successfully booted up. Startup took ");
  Serial.print(millis() - start_time);
  Serial.println("ms");

  // arduino bootloader now starts with calling loop() over and over again as soon as we return
}

// Marks this program as shut down, shuts down the servo.
// Please note that does *NOT* actually terminate the program
// (as it is impossible to turn an Arduino off by itself)!
void shutdown() {
  // mark program as shut down
  is_shutdown = true;

  // Shutdown the rotors
  rotor_1.write(0);
  rotor_2.write(0);
  rotor_3.write(0);
  rotor_4.write(0);

  Serial.println("[EMERGENCY] Shutdown!");
}

// Snaps and return the value of 50 if it is inside a range
// that is considered to ease behavior (rounding errors, minor differences
// from remote control and known range).
// Otherwise returns given value.
int maybeSnapNeutral(int value) {
  // mark everything 45..=54 as 50
  if (value >= 45 && value <= 54) {
    return 50;
  }

  return value;
}

// Reads and returns a raw PWM(pulse-width modulation) value from the given `channel_id` (i.e. the output port a cable
// should be connected to).
unsigned long readValue(int channel_id) {
  return pulseIn(channel_id, HIGH, PULSE_IN_TIMEOUT);
}

unsigned char getThrustInPercent(unsigned long rawValue) {
  unsigned long mappedValue = map(rawValue, 1025, 1874, PERCENT_LOW, PERCENT_TOP);

  return min(mappedValue, 100);
}

// This function extracts the value like this:
// 0% -> backwards
// 50% -> steady
// 100% -> forwards
int getMovementForwardBackwardInPercent(unsigned long rawValue) {
  unsigned long mappedValue = map(rawValue, 1132, 1914, PERCENT_LOW, PERCENT_TOP);

  return min(maybeSnapNeutral(mappedValue), 100);
}

// This function extracts the value like this:
// 0% -> left
// 50% -> steady
// 100% -> right
int getMovementLeftRightInPercent(unsigned long rawValue) {
  unsigned long mappedValue = map(rawValue, 1128, 1887, PERCENT_LOW, PERCENT_TOP);

  return min(maybeSnapNeutral(mappedValue), 100);
}

// This function extracts the value like this:
// 0% -> left
// 50% -> steady
// 100% -> right
int getRotationLeftRightInPercent(unsigned long rawValue) {
  unsigned long mappedValue= map(rawValue, 1098, 1872, PERCENT_LOW, PERCENT_TOP);

  return min(maybeSnapNeutral(mappedValue), 100);
}


// Helper utility function to avoid clutter-code in the main function
// that logs:
//  * a) the name this value stands for(the first character should be upper-cased),
//  * b) the value itself and
//  * c) the raw value (we fed our algorithms with)
// so they are printed out on the Serial Monitor(if connected).
void logValue(unsigned long value, String name, unsigned long rawValue) {
  Serial.print(name);
  Serial.print(" value: ");

  Serial.print(value);

#ifdef VERBOSE
  Serial.print("% (");
  Serial.print(rawValue);
  Serial.println(")");
#else
  Serial.println("%");
#endif // VERBOSE
}

// Writes the given thrust value to the rotors if and only if (iff) the value is new
// (avoiding spamming them with time-consuming commands).
void writeThrustIffNew(int thrustValue) {
  if (thrustValue != last_thrust_value) {
#ifdef VERBOSE
    Serial.print("New thrust value: ");
    Serial.println(thrustValue);
#endif // VERBOSE
    last_thrust_value = thrustValue;
  } else {
    // same thrust, nothing to do (saving time)
    return;
  }

  rotor_1.write(thrustValue);
  rotor_2.write(thrustValue);
  rotor_3.write(thrustValue);
  rotor_4.write(thrustValue);
}

void loop() {
  if (is_shutdown) {
    // sleep for a little while to avoid busy loop as we cannot (really) shut the arduino down anyhow
    delay(1000);
    return;
  }

  unsigned long tickStartTime = millis();

  // horizontal (left/right)
  int channel1 = readValue(CHANNEL_1_PORT);
  // horizontal movement (forward/backward)
  int channel2 = readValue(CHANNEL_2_PORT);
  // vertical movement (thrust -> up/down)
  int channel3 = readValue(CHANNEL_3_PORT);
  // rotation own axis
  int channel4 = readValue(CHANNEL_4_PORT);

  ValueSet values = {
    getThrustInPercent(channel3),
    getMovementForwardBackwardInPercent(channel2),
    getMovementLeftRightInPercent(channel1),
    getRotationLeftRightInPercent(channel4)
  };

  logValue(values.thrust, "Thrust", channel3);
  logValue(values.rotation_l_r, "Rotation (left/right)", channel4);
  logValue(values.movement_l_r, "Movement (left/right)", channel1);
  logValue(values.movement_f_b, "Movement (forward/backward)", channel2);

  writeThrustIffNew(values.thrust);

  unsigned long tickDuration = millis() - tickStartTime;

#ifdef VERBOSE
  Serial.print("Tick took ");
  Serial.print(tickDuration);
  Serial.println("ms");
#endif // VERBOSE

  // sleep for as long as we need to have a consistent tick time of ROUGH_TICK_TIME ms.

  // if we already took ROUGH_TICK_TIME ms(which should never happen lol), we abort asafp
  if (tickDuration >= ROUGH_TICK_TIME) {
    Serial.println("[WARNING] Tick duration was higher than ROUGH_TICK_TIME!");
    Serial.println();
    return;
  } else {
    // only do cleanup tasks if we have some greater timespan to invest into.
    bool cleanUpTaskWorth = (ROUGH_TICK_TIME - tickDuration) >= 50;

    if (cleanUpTaskWorth) {
      Serial.println("Doing cleanup tasks.");
      // Cleanup tasks, they should not take too much time though

      // TODO: Add cleanup tasks

      // tick_duration is lower than ROUGH_TICK_TIME now, thus recalculate it
      unsigned long tickDuration = millis() - tickStartTime;

      // check whether the cleanup task took too long
      if (tickDuration >= ROUGH_TICK_TIME) {
        Serial.println("[WARNING] \"Cleanup task\" took too much time!");
        Serial.println();
        return;
      }
    }

    // calculate how much time we need to sleep now
    unsigned long sleepTime = ROUGH_TICK_TIME - (millis() - tickStartTime);

    // ignore overflows/underflows
    if (sleepTime <= 0 || sleepTime >= 1000) {
      Serial.println("[WARNING] Underflow or overflow detected!");
      return;
    }

#ifdef VERBOSE
    Serial.print("Sleeping for ");
    Serial.print(sleepTime);
    Serial.println("ms… ");
#endif // VERBOSE

    delay(sleepTime);

#ifdef VERBOSE
    Serial.println("Woked up");
#endif // VERBOSE

    Serial.println(); // make some room for the next output season
  }
}
