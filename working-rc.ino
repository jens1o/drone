#include<Servo.h>

// Defines whether there is verbose logging on the Serial Monitor
// commented out = production
// not commented out = debug mode
#define VERBOSE

//#define CLEANUP_BATTERY_CHECK

#ifdef CLEANUP_BATTERY_CHECK
#define CLEANUP_BATTERY_CHECK_PORT A0
#endif // CLEANUP_BATTERY_CHECK

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
#define ROTOR_1_MAX_STRENGTH 2400

#define ROTOR_2_MIN_STRENGTH 990 // (60)
#define ROTOR_2_MAX_STRENGTH 2500

#ifdef CLEANUP_BATTERY_CHECK
// This class provides functions for checking whether we have enough electrical power,
// and avoid (permanently) harming the battery if it is going to be empty soon.
class PowerManager
{
  private:
    float voltage_value_;
    unsigned long last_battery_check_ = 0;

  public:
    // Reads and converts the raw value from the power supply and stores
    // it(can be recieved by `GetVoltage()`).
    virtual void ReadVoltage()
    {
      this->last_battery_check_ = millis();
#ifdef VERBOSE
      Serial.println("Reading new voltage value from power supply!");
#endif // VERBOSE

      int rawValue = analogRead(CLEANUP_BATTERY_CHECK_PORT);

      this->voltage_value_ = rawValue * (5.00 / 1023.00) * 2;
#ifdef VERBOSE
      Serial.print("Battery voltage: ");
      Serial.print(this->voltage_value_);
      Serial.println("V");
#endif // VERBOSE
    }

    // Checks whether the value hold in cache is still up to date
    // (avoiding a persistent leak of time)
    virtual void MaybeRefreshCache()
    {
      // check whether the last read was (at least) 30 seconds ago
      bool shouldRefreshCache = ((millis() - this->last_battery_check_) >= 30 * 1000);

      if (shouldRefreshCache) {
        // update cache by reading the value and save it
        this->ReadVoltage();
      }
    }

    virtual float GetVoltage()
    {
      // check whether we need to read the voltage again because of new conditions
      this->MaybeRefreshCache();

      return this->voltage_value_;
    }

    virtual bool PowerSupplyIsOkay()
    {
      float voltageValue = this->GetVoltage();

      if (voltageValue <= 6.50) {
        Serial.println("[WARNING] Power supply is too low!");
        return false; // TODO: Change to false in production
      }

      return true;
    }
};
#endif // CLEANUP_BATTERY_CHECK

struct ValueSet
{
  int thrust;
  int movement_f_b;
  int movement_l_r;
  int rotation_l_r;
};

#ifdef CLEANUP_BATTERY_CHECK
PowerManager *power_manager;
#endif // CLEANUP_BATTERY_CHECK

unsigned long start_time;

int last_thrust_value = 0;

int const INPUT_PORTS[] = {RANDOM_SEED_PORT, CHANNEL_1_PORT, CHANNEL_2_PORT, CHANNEL_3_PORT, CHANNEL_4_PORT};
// in the arduino world, there is no reliable way of knowing how long an array is, thus computing this at compile time!
#define kInputPortLength (sizeof(INPUT_PORTS)/sizeof(int))


bool is_shutdown = false;

Servo rotor_1, rotor_2;

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

  // seed the (pseudo-)random number generator asap on an **unconnected** pin
  randomSeed(analogRead(RANDOM_SEED_PORT));

  // Connect to a (potential) computer for outputting debug information on baud 9600
  Serial.begin(9600);

  // Set output pins
  rotor_1.attach(ROTOR_1_PORT, ROTOR_1_MIN_STRENGTH, ROTOR_1_MAX_STRENGTH);
  rotor_2.attach(ROTOR_2_PORT, ROTOR_2_MIN_STRENGTH, ROTOR_2_MAX_STRENGTH);

  for (int i = 0; i <= 50; i++) {
    rotor_1.write(i);
    rotor_2.write(i);
    Serial.print("Current i: ");
    Serial.println(i);
    delay(250);
  }

  delay(300);
  shutdown();
  return;

  //rotor_2 = new Rotor(ROTOR_2_PORT, ROTOR_2_MIN_STRENGTH, ROTOR_2_MAX_STRENGTH);

#ifdef CLEANUP_BATTERY_CHECK
  power_manager = new PowerManager();

  if (!power_manager->PowerSupplyIsOkay()) {
    shutdown();
    return;
  }
#endif // CLEANUP_BATTERY_CHECK

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

  // Shutdown the servos
  rotor_1.write(0);
  rotor_2.write(0);

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
  unsigned long mappedValue = map(rawValue, 1040, 1874, PERCENT_LOW, PERCENT_TOP);

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
#ifdef CLEANUP_BATTERY_CHECK
      if (!power_manager->PowerSupplyIsOkay()) {
        shutdown();
        return;
      }
#endif // CLEANUP_BATTERY_CHECK

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
