#include<Servo.h>

// Defines whether there is verbose logging on the Serial Monitor
// commented out = production
// not commented out = debug mode
#define VERBOSE

#define CLEANUP_BATTERY_CHECK

#ifdef CLEANUP_BATTERY_CHECK
#define CLEANUP_BATTERY_CHECK_PORT A0
#endif

#define PERCENT_TOP 100
#define PERCENT_LOW 0
// MUST NOT be too less, otherwise we will get weird values
#define PULSE_IN_TIMEOUT 25000
// time a tick should roughly take.
#define ROUGH_TICK_TIME 500 // 225

// It is important that this port is not connected to anything at all!
#define RANDOM_SEED_PORT 12

#define SERVO_1_PORT 9
#define SERVO_2_PORT 10
#define SERVO_3_PORT 11
#define SERVO_4_PORT 12

#define CHANNEL_1_PORT 5
#define CHANNEL_2_PORT 8
#define CHANNEL_3_PORT 7
#define CHANNEL_4_PORT 6

#define MIN_ARM_STRENGTH 55 // (60)
#define MAX_ARM_STRENGTH 100

unsigned long START_TIME = 0;

int last_thrust_value = 0;

int INPUT_PORTS[] = {CHANNEL_1_PORT, CHANNEL_2_PORT, CHANNEL_3_PORT, CHANNEL_4_PORT};

int SERVO_PORTS[] = {SERVO_1_PORT};

bool IS_SHUTDOWN = false;

Servo servo_1;

Servo servos[] = {servo_1};

#ifdef CLEANUP_BATTERY_CHECK
class BatteryManager
{
  private:
    float voltage_value;
    unsigned long last_battery_check = START_TIME;

  public:
    virtual void readVoltage() {
      this->last_battery_check = millis();
#ifdef VERBOSE
      Serial.println("Need to refresh!");
#endif

      int raw_value = analogRead(CLEANUP_BATTERY_CHECK_PORT);

      this->voltage_value = raw_value * (5.00 / 1023.00) * 2;
#ifdef VERBOSE
      Serial.print("Battery voltage: ");
      Serial.print(voltage_value);
      Serial.println("V");
#endif
    }

    virtual bool needNewValue() {
      // check whether the last read was (at least) 30 seconds ago
      if ((millis() - this->last_battery_check) >= 30 * 1000) {
        return true;
      }

      return false;
    }

    virtual float getVoltage() {
      // check whether we need to read the voltage again because of new conditions
      if (this->needNewValue()) {
        // and if we, read the value
        this->readVoltage();
      }

      return this->voltage_value;
    }

    virtual bool isBatteryOk() {
      float voltage_value = this->getVoltage();

      if (voltage_value <= 6.50) {
        return true; // TODO: Change for production
      }

      return true;
    }
};
#endif

struct ValueSet {
  unsigned char thrust;
  unsigned char movement_f_b;
  unsigned char movement_l_r;
  unsigned char rotation_l_r;
};

#ifdef CLEANUP_BATTERY_CHECK
BatteryManager *btrMgr;
#endif

void setup() {
  // should be close to zero, otherwise the bootloader is doing quite crazy stuff?
  START_TIME = millis();

  // Connect to computer for outputting debug information on baud 9600
  Serial.begin(9600);

  // seed the (pseudo-)random number generator
  randomSeed(RANDOM_SEED_PORT);

  // Set output pins
  servo_1.attach(SERVO_1_PORT);

  // Initalize input pins for reading values from them
  for (int i = 0; i < sizeof(INPUT_PORTS); i++) {
    pinMode(INPUT_PORTS[i], INPUT);
  }

#ifdef CLEANUP_BATTERY_CHECK
  btrMgr = new BatteryManager();

  if (!btrMgr->isBatteryOk()) {
    shutdown();
    return;
  }
#endif

  Serial.print("Successfully booted up. Startup took");
  Serial.print(millis() - START_TIME);
  Serial.println("ms");

  // arduino bootloader now starts with calling loop() over and over again as soon as we return
}
void shutdown() {
  IS_SHUTDOWN = true;

  // Shutdown the servos
  for (int i = 0; i < sizeof(servos); i++) {
    servos[i].write(MIN_ARM_STRENGTH);
  }

  Serial.println("[EMERGENCY] Shutdown!");
}

int check_if_neutral(int to_round) {
  if (to_round >= 45 && to_round <= 54) {
    return 50;
  }

  return to_round;
}

// int round_down_to_next_10(int to_round) {
//  return to_round - to_round % 10;
// }

unsigned long read_value(int channel_id) {
  return pulseIn(channel_id, HIGH, PULSE_IN_TIMEOUT);
}

unsigned char get_thrust_in_percent(unsigned long raw_value) {
  unsigned long mapped_value = map(raw_value, 1040, 1874, PERCENT_LOW, PERCENT_TOP);

  return min(mapped_value, 100);
}

// This function extracts the value like this:
// 0% -> backwards
// 50% -> steady
// 100% -> forwards
unsigned long get_movement_forward_backward_in_percent(unsigned long raw_value) {
  unsigned long mapped_value = map(raw_value, 1132, 1914, PERCENT_LOW, PERCENT_TOP);
  unsigned int rounded_value = check_if_neutral(mapped_value);

  return min(rounded_value, 100);
}

// This function extracts the value like this:
// 0% -> left
// 50% -> steady
// 100% -> right
unsigned int get_movement_left_right_in_percent(unsigned long raw_value) {
  unsigned int mapped_value = map(raw_value, 1128, 1887, PERCENT_LOW, PERCENT_TOP);
  unsigned int rounded_value = check_if_neutral(mapped_value);

  return min(rounded_value, 100);
}

// This function extracts the value like this:
// 0% -> left
// 50% -> steady
// 100% -> right
unsigned long get_rotation_left_right_in_percent(unsigned long raw_value) {
  unsigned long mapped_value = map(raw_value, 1098, 1872, PERCENT_LOW, PERCENT_TOP);
  unsigned int rounded_value = check_if_neutral(mapped_value);

  return min(rounded_value, 100);
}


void log_values(unsigned long value, String name, unsigned long raw_value) {
  Serial.print(name);
  Serial.print(" value: ");

  Serial.print(value);

#ifdef VERBOSE
  Serial.print("% (");
  Serial.print(raw_value);
  Serial.println(")");
#else
  Serial.println("%");
#endif
}

int calculate_thrust_output(unsigned long raw_value) {
  return map(raw_value, 0, 100, MIN_ARM_STRENGTH, MAX_ARM_STRENGTH);
}

void set_thrust_if_changed(int thrust_value) {
  if (thrust_value != last_thrust_value) {
#ifdef VERBOSE
    Serial.print("New thrust value: ");
    Serial.println(thrust_value);
#endif
    last_thrust_value = thrust_value;
  } else {
    // same thrust, nothing to do
    return;
  }

  servo_1.write(thrust_value);
}

void loop() {
  if (IS_SHUTDOWN) {
    // sleep for a little while to avoid busy loop as we cannot (really) shut the arduino down anyhow
    delay(1000);
    return;
  }

  unsigned long tick_start_time = millis();

  // horizontal (left/right)
  int ch1 = read_value(CHANNEL_1_PORT);
  // horizontal movement (forward/backward)
  int ch2 = read_value(CHANNEL_2_PORT);
  // vertical movement (thrust -> up/down)
  int ch3 = read_value(CHANNEL_3_PORT);
  // rotation own axis
  int ch4 = read_value(CHANNEL_4_PORT);

  ValueSet values = {
    get_thrust_in_percent(ch3),
    get_movement_forward_backward_in_percent(ch2),
    get_movement_left_right_in_percent(ch1),
    get_rotation_left_right_in_percent(ch4)
  };

  log_values(values.thrust, "Thrust", ch3);

  log_values(values.rotation_l_r, "Rotation (left/right)", ch4);

  log_values(values.movement_l_r, "Movement (left/right)", ch1);

  log_values(values.movement_f_b, "Movement (forward/backward)", ch2);

  set_thrust_if_changed(calculate_thrust_output(values.thrust));

  unsigned long tick_duration = millis() - tick_start_time;


#ifdef VERBOSE
  Serial.print("Tick took ");
  Serial.print(tick_duration);
  Serial.println("ms");
#endif

  // sleep for as long as we need to have a consistent tick time of ROUGH_TICK_TIME ms.

  // if we already took ROUGH_TICK_TIME ms(which should never happen lol), we abort asafp
  if (tick_duration >= ROUGH_TICK_TIME) {
    Serial.println("[WARNING] Tick duration was higher than ROUGH_TICK_TIME!");
    Serial.println();
    return;
  } else {
    // only do cleanup tasks if we have some greater timespan to invest into.
    bool cleanUpTaskWorth = (ROUGH_TICK_TIME - tick_duration) >= 50;

    if (cleanUpTaskWorth) {
      Serial.println("Doing cleanup tasks.");
      // Cleanup tasks, they should not take too much time though
#ifdef CLEANUP_BATTERY_CHECK
      if (!btrMgr->isBatteryOk()) {
        shutdown();
        return;
      }
#endif

      // tick_duration is lower than ROUGH_TICK_TIME now, thus recalculate it
      unsigned long tick_duration = millis() - tick_start_time;

      // check whether the cleanup task took too long
      if (tick_duration >= ROUGH_TICK_TIME) {
        Serial.println("[WARNING] \"Cleanup task\" took too much time!");
        Serial.println();
        return;
      }
    }

    // calculate how much time we need to sleep now
    unsigned long sleep_time = ROUGH_TICK_TIME - (millis() - tick_start_time);

    // ignore overflows/underflows
    if (sleep_time <= 0 || sleep_time >= 1000) {
      Serial.println("[WARNING] Underflow or overflow detected!");
      return;
    }

#ifdef VERBOSE
    Serial.print("Sleeping for ");
    Serial.print(sleep_time);
    Serial.println("ms… ");
#endif

    delay(sleep_time);

#ifdef VERBOSE
    Serial.println("Woked up");
#endif

    Serial.println(); // make some room for the next output season
  }
}
