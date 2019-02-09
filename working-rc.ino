#include<Servo.h>

// Defines whether there is verbose logging on the Serial Monitor
#define VERBOSE 1

#define PERCENT_TOP 100
#define PERCENT_LOW 0
#define PULSE_IN_TIMEOUT 25000
// time a tick should roughly take.
#define ROUGH_TICK_TIME 225



#define SERVO_1_PORT 2

#define MIN_ARM_STRENGTH 55 // (60)
#define MAX_ARM_STRENGTH 100

unsigned long START_TIME;

int LAST_THRUST_VALUE = 0;

Servo servo_1;

struct ValueSet {
  unsigned char thrust;
  unsigned char movement_f_b;
  unsigned char movement_l_r;
  unsigned char rotation_l_r;
};

void setup() {
  START_TIME = millis();

  // Set output pins
  servo_1.attach(SERVO_1_PORT);

  // Set input pins
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);

  // Connect to computer for outputting debug information
  Serial.begin(9600);

  Serial.print("Successfully booted up. Current time(should be close to zero), startup time:");
  Serial.println(millis() - START_TIME);
}

int round_up_to_next_10(int to_round) {
  if (to_round % 10 == 0) return to_round;
  return (10 - to_round % 10) + to_round;
}

// int round_down_to_next_10(int to_round) {
//  return to_round - to_round % 10;
// }

unsigned long read_value(int channel_id) {
  return pulseIn(channel_id, HIGH, PULSE_IN_TIMEOUT);
}

unsigned char get_thrust_in_percent(unsigned long raw_value) {
  unsigned long mapped_value = map(raw_value, 1060, 1874, PERCENT_LOW, PERCENT_TOP);
  //unsigned char rounded_value = round_up_to_next_10(mapped_value);

  return min(mapped_value, 100);
}

// This function extracts the value like this:
// 0% -> backwards
// 50% -> steady
// 100% -> forwards
unsigned long get_movement_forward_backward_in_percent(unsigned long raw_value) {
  unsigned long mapped_value = map(raw_value, 1132, 1914, PERCENT_LOW, PERCENT_TOP);
  unsigned int rounded_value = round_up_to_next_10(mapped_value);

  return min(rounded_value, 100);
}

// This function extracts the value like this:
// 0% -> left
// 50% -> steady
// 100% -> right
unsigned int get_movement_left_right_in_percent(unsigned long raw_value) {
  unsigned int mapped_value = map(raw_value, 1128, 1887, PERCENT_LOW, PERCENT_TOP);
  unsigned int rounded_value = round_up_to_next_10(mapped_value);

  return min(rounded_value, 100);
}

// This function extracts the value like this:
// 0% -> left
// 50% -> steady
// 100% -> right
unsigned long get_rotation_left_right_in_percent(unsigned long raw_value) {
  unsigned long mapped_value = map(raw_value, 1105, 1867, PERCENT_LOW, PERCENT_TOP);
  unsigned int rounded_value = round_up_to_next_10(mapped_value);

  return min(rounded_value, 100);
}


void log_values(unsigned long value, String name, unsigned long raw_value) {
  Serial.print("Current ");
  Serial.print(name);
  Serial.print(" value: ");

  Serial.print(value);

  if (VERBOSE) {
    Serial.print("% (");
    Serial.print(raw_value);
    Serial.println(")");
  } else {
    Serial.println("%");
  }
}

int calculate_thrust(unsigned long raw_value) {
  return map(raw_value, 0, 100, MIN_ARM_STRENGTH, MAX_ARM_STRENGTH);
}

void loop() {
  unsigned long tick_start_time = millis();

  // horizontal (left/right)
  int ch1 = read_value(5);
  // horizontal movement (forward/backward)
  int ch2 = read_value(8);
  // vertical movement (thrust -> up/down)
  int ch3 = read_value(7);
  // rotation own axis
  int ch4 = read_value(6);

  ValueSet values = {
    get_thrust_in_percent(ch3),
    get_movement_forward_backward_in_percent(ch2),
    get_movement_left_right_in_percent(ch1),
    get_rotation_left_right_in_percent(ch4)
  };

  log_values(values.thrust, "Thrust", ch3);

  int new_thrust = calculate_thrust(values.thrust);

  if (new_thrust != LAST_THRUST_VALUE) {
    Serial.print("New thrust value: ");
    Serial.println(new_thrust);
    LAST_THRUST_VALUE = new_thrust;
    servo_1.write(LAST_THRUST_VALUE);
  }


  log_values(values.rotation_l_r, "Rotation (left/right)", ch4);

  log_values(values.movement_l_r, "Movement (left/right)", ch1);

  log_values(values.movement_f_b, "Movement (forward/backward)", ch2);

  unsigned long tick_duration = millis() - tick_start_time;

  Serial.print("Tick took ");
  Serial.print(tick_duration);
  Serial.println("ms");


  // sleep for as long as we need to have a consistent tick time of 200ms.

  // if we already took ROUGH_TICK_TIME ms(which should never happen lol), we abort asafp
  if (tick_duration >= ROUGH_TICK_TIME) {
    Serial.println("[WARNING] Tick duration was higher than ROUGH_TICK_TIME!");
    Serial.println();
    return;
  } else {
    // tick_duration is lower than ROUGH_TICK_TIME now

    // calculate how much time we need to sleep
    unsigned long sleep_time = ROUGH_TICK_TIME - tick_duration;
    Serial.print("Sleeping for ");
    Serial.print(sleep_time);
    Serial.println("ms… ");

    delay(sleep_time);
    Serial.println("Woked up");
    Serial.println(); // make some room for the next output season
  }
}
