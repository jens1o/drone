#define PERCENT_TOP 100
#define PERCENT_LOW 0
#define PULSE_IN_TIMEOUT 25000

struct ValueSet {
  unsigned char thrust;
  unsigned char movement_f_b;
  unsigned char movement_l_r;
  unsigned char rotation_l_r;
};

void setup() {
  // Set input pins
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);

  // Connect to computer for outputting debug information
  Serial.begin(9600);
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
  unsigned long mapped_value = map(raw_value, 1098, 1874, PERCENT_LOW, PERCENT_TOP);
  unsigned char rounded_value = round_up_to_next_10(mapped_value);

  return min(rounded_value, 100);
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

void loop() {
  // horizontal (left/right)
  int ch1 = read_value(5);
  // horizontal movement (forward/backward)
  int ch2 = read_value(8);
  // horizontal rotation (thrust -> up/down)
  int ch3 = read_value(7);
  // rotation own axis
  int ch4 = read_value(6);

  ValueSet values = {
    get_thrust_in_percent(ch3),
    get_movement_forward_backward_in_percent(ch2),
    get_movement_left_right_in_percent(ch1),
    get_rotation_left_right_in_percent(ch4)
  };

  Serial.print("Current thrust: ");
  Serial.print(values.thrust);
  Serial.print("% (");
  Serial.print(ch3);
  Serial.println(")");

  Serial.print("Current rotation value: ");
  Serial.print(values.rotation_l_r);
  Serial.print("% (");
  Serial.print(ch4);
  Serial.println(")");

  Serial.print("Current left/right value: ");
  Serial.print(values.movement_l_r);
  Serial.print("% (");
  Serial.print(ch1);
  Serial.println(")");

  Serial.print("Current movement (forward/backward): ");
  Serial.print(values.movement_f_b);
  Serial.print("% (");
  Serial.print(ch2);
  Serial.println(")");

  Serial.println(); // make some room for the next output season

  delay(2000);
}
