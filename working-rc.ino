// Defines whether there is verbose logging on the Serial Monitor
// commented out = production
// not commented out = debug mode
#define VERBOSE

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// #define CLEANUP_BATTERY_CHECK
#define CLEANUP_BATTERY_CHECK_PORT A0
#define CLEANUP_BATTERY_CHECK_INTERVAL (10 * 1000)

#define PERCENT_TOP 100
#define PERCENT_LOW 0
// MUST NOT be too less, otherwise we will get weird values
#define PULSE_IN_TIMEOUT 25000
// time a tick should roughly take.
#define ROUGH_TICK_TIME 350 // 225

// It is important that this port is not connected to anything at all!
#define RANDOM_SEED_PORT 15

#define ROTOR_1_PORT 9
#define ROTOR_2_PORT 10
#define ROTOR_3_PORT 11
#define ROTOR_4_PORT 12

#define CHANNEL_1_PORT 5
#define CHANNEL_2_PORT 6
#define CHANNEL_3_PORT 7
#define CHANNEL_4_PORT 8

#define LEFT 0
#define RIGHT 1

#define ROTOR_1_MIN_STRENGTH 995
#define ROTOR_1_MAX_STRENGTH 2945
#define ROTOR_1_DIRECTION RIGHT

#define ROTOR_2_MIN_STRENGTH 1025
#define ROTOR_2_MAX_STRENGTH 3075
#define ROTOR_2_DIRECTION LEFT

#define ROTOR_3_MIN_STRENGTH 995
#define ROTOR_3_MAX_STRENGTH 2856
#define ROTOR_3_DIRECTION RIGHT

#define ROTOR_4_MIN_STRENGTH 975
#define ROTOR_4_MAX_STRENGTH 2716
#define ROTOR_4_DIRECTION LEFT

#define FREQ 250             // Sampling frequency
#define SSF_GYRO 65.5        // Sensitivity Scale Factor of the gyrometer from the datasheet
#define GYRO_INTERRUPT_PIN 2 // use pin 2 on Arduino Uno and Arduino Mega boards

#define X 0 // X axis
#define Y 1 // Y axis
#define Z 2 // Z axis

#define YAW 0
#define PITCH 1
#define ROLL 2

struct RawValueSet
{
  unsigned long thrust;
  unsigned long movement_f_b;
  unsigned long movement_l_r;
  unsigned long rotation_l_r;
};

struct AccelerometerMeasurements
{
  // The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
  int acc_raw[3] = {0, 0, 0};
  // The RAW values got from gyro (in °/sec) in that order: X, Y, Z
  int gyro_raw[3] = {0, 0, 0};
};

struct AccelerometerResults
{
  // The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
  int acc_raw[3] = {0, 0, 0};
  // Calculated angles from accelerometer's values in that order: X, Y, Z
  VectorInt16 acc_angle;
  int temperature;
  /**
     Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
      - Left wing up implies a positive roll
      - Nose up implies a positive pitch
      - Nose right implies a positive yaw
  */
  float measures[3] = {0, 0, 0};
  // Total 3D acceleration vector in m/s²
  long acc_total_vector;
};

unsigned long start_time;

int last_thrust_value = 0;

int const INPUT_PORTS[] = {RANDOM_SEED_PORT, CHANNEL_1_PORT, CHANNEL_2_PORT, CHANNEL_3_PORT, CHANNEL_4_PORT};
// in the arduino world, there is no reliable way of knowing how long an array is, thus computing this at compile time
#define kInputPortLength (sizeof(INPUT_PORTS) / sizeof(int))

int const OUTPUT_PORTS[] = {ROTOR_1_PORT, ROTOR_2_PORT, ROTOR_3_PORT, ROTOR_4_PORT};
// in the arduino world, there is no reliable way of knowing how long an array is, thus computing this at compile time
#define kOutputPortLength (sizeof(OUTPUT_PORTS) / sizeof(int))

Servo rotor_1, rotor_2, rotor_3, rotor_4;

const int MPU_ADDRESS = 0x68; // I2C address of the MPU-6050

// DO NOT USE this variable outside of AccelerationController,
// it must be placed in the global scope as the Arduino programming language(Processing) is
// unable to process static class variables … <.>
// Do not leave out the `volatile`, as it is needed here(although heavily decreasing the
// abilities of the optimizer)!
volatile bool __acc_controller_has_new_data = false;

// Responsible for reading and calculating (useful) data from the MPU-6050
class AccelerationController
{
private:
  MPU6050 _mpu;
  AccelerometerResults _last_results;
  bool _init_successful = true;
  // holds received interrupt status byte
  uint8_t _mpuInterruptStatus;
  // count of all bytes currently in FIFO buffer
  uint16_t _fifoBufferLength;
  // FIFO storage buffer that is being filled as soon as data is received after an interrupt from the MPU-6050
  // "128 bits ought be enough for everybody"
  uint8_t _fifoBuffer[128];
  uint8_t _mpuInterruptPacketSize;

  static void _hasDataReadyCallback()
  {
    __acc_controller_has_new_data = true;
  }

public:
  // Initalizes the gyro to retrieve data from it in the future
  AccelerationController(void)
  {
    unsigned long initStartTime = millis();

    // start listening
    Wire.begin();
    // set I2C clock to 400 kHz
    Wire.setClock(400000);

    // Order the MPU(-library) to initalize itself
    this->_mpu.initialize();

    // first initalize the mpu, then init the pin on the board
    pinMode(GYRO_INTERRUPT_PIN, INPUT);

    // Test connection to verify it can work
    if (!this->_mpu.testConnection())
    {
      _init_successful = false;
      Serial.println("[EMERGENCY] [AccelerationController] MPU-6050 connection failed!");
      return;
    }

    // HACK: Wait until the MPU is initalized
    delay(200);

    uint8_t deviceStatus = this->_mpu.dmpInitialize();

    // check for errors
    if (deviceStatus != 0)
    {
      this->_init_successful = false;
      Serial.println("[WARNING] [AccelerationController] Non-zero error code when initalizing MPU!");
      return;
    }

    // calibrated using a compass and try and error
    this->_mpu.setXGyroOffset(220);
    this->_mpu.setYGyroOffset(76);
    this->_mpu.setZGyroOffset(-85);
    this->_mpu.setZAccelOffset(1788);

    this->_mpu.setDMPEnabled(true);

    // initalize interrupt for receiving continuous data
    attachInterrupt(digitalPinToInterrupt(GYRO_INTERRUPT_PIN), _hasDataReadyCallback, RISING);
    // remember error-code
    this->_mpuInterruptStatus = this->_mpu.getIntStatus();
    // save for later comparision
    this->_mpuInterruptPacketSize = this->_mpu.dmpGetFIFOPacketSize();

    Serial.print("[INFO] [AccelerationController] Initalized. Took ");
    Serial.print(millis() - initStartTime);
    Serial.println("ms");
  }

  // Returns true iff(if and only if) the intialization process was succesful
  bool initSuccessful()
  {
    return this->_init_successful;
  }

  // Blocking until it receives updated data from the gyro to make sure
  // that there is always relevant (current) data
  void Tick()
  {
    // I miss you, exceptions.
    // use variable directly over the `initSuccessful`-function because the compiler apparently
    // isn't able to do inlining |>_<|
    if (!this->_init_successful)
      return;

    // wait for MPU interrupt or extra packet(s) available
    while (!__acc_controller_has_new_data && this->_fifoBufferLength < this->_mpuInterruptPacketSize)
    {
      if (__acc_controller_has_new_data && this->_fifoBufferLength < this->_mpuInterruptPacketSize)
      {
        // try to get out of the infinite loop
        this->_fifoBufferLength = this->_mpu.getFIFOCount();
      }
      // continue
    }
    // reset to ready for further signals
    __acc_controller_has_new_data = false;

    this->_mpuInterruptStatus = this->_mpu.getIntStatus();
    // get current FIFO count
    this->_fifoBufferLength = this->_mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((this->_mpuInterruptStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || this->_fifoBufferLength >= 1024)
    {
      // reset so we can continue cleanly
      this->_mpu.resetFIFO();
      this->_fifoBufferLength = this->_mpu.getFIFOCount();
      Serial.println("[WARNING] [AccelerationController] FIFO overflow!");
      return;
    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)

    if (!(this->_mpuInterruptStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
      return;
    } 
      // wait for correct available data length, should be a VERY short wait
    while (this->_fifoBufferLength < this->_mpuInterruptPacketSize) {
      this->_fifoBufferLength = this->_mpu.getFIFOCount();
    }

    // read a packet from FIFO
    this->_mpu.getFIFOBytes(this->_fifoBuffer, this->_mpuInterruptPacketSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt) -> performance win!
    this->_fifoBufferLength -= this->_mpuInterruptPacketSize;

    struct AccelerometerResults results;

    this->_mpu.dmpGetAccel(&results.acc_angle, this->_fifoBuffer);

    Serial.print("ACC angles:\tX:\t");
    Serial.print(results.acc_angle.x);
    Serial.print("\tY:\t");
    Serial.print(results.acc_angle.y);
    Serial.print("\tZ:\t");
    Serial.println(results.acc_angle.z);

    // get gyro data
    int16_t gyro[3];

    this->_mpu.dmpGetGyro(gyro, this->_fifoBuffer);

    Serial.print("Gyro angles:\tX:\t");
    Serial.print(gyro[X]);
    Serial.print("\tY:\t");
    Serial.print(gyro[Y]);
    Serial.print("\tZ:\t");
    Serial.println(gyro[Z]);

    // Calculated angles from gyro's values in that order: X, Y, Z
    int32_t gyro_angle[3];

    gyro_angle[X] = gyro[X];
    gyro_angle[Y] = gyro[Y];
    gyro_angle[Z] = -gyro[Z] / SSF_GYRO;

    // Angle calculation using integration
    gyro_angle[X] += (gyro[X] / (FREQ * SSF_GYRO));
    gyro_angle[Y] += (-gyro[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if it has yawed
    results.measures[Y] += gyro[X] * sin(gyro[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    results.measures[X] -= gyro_angle[Y] * sin(gyro[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    results.measures[Z] = gyro_angle[Z]; // Store the angular motion for this axis

    this->_last_results = results;
  }

  // returns the last results that have been processed successfully
  AccelerometerResults GetResults()
  {
    return this->_last_results;
  }
};

// Responsible for deciding how strong the rotors should be rotating
class FlightController
{
private:
  byte _relative_1 = 0;
  byte _relative_2 = 0;
  byte _relative_3 = 0;
  byte _relative_4 = 0;
  byte _relative_thrust = 0;
  // PITCH
  byte _relative_movement_f_b = 0;
  // ROLL
  byte _relative_movement_l_r = 0;
  // YAW
  byte _relative_rotation_l_r = 0;
  bool _init_successful = true;
  Servo _servo_1;
  Servo _servo_2;
  Servo _servo_3;
  Servo _servo_4;
  float _Kp[3] = {4.0, 1.3, 1.3};       // P coefficients in that order : Yaw, Pitch, Roll
  float _Ki[3] = {0.02, 0.04, 0.04};    // I coefficients in that order : Yaw, Pitch, Roll
  float _Kd[3] = {0, 18, 18};           // D coefficients in that order : Yaw, Pitch, Roll
  float _error_sum[3] = {0, 0, 0};      // Error sums (used for integral component) : [Yaw, Pitch, Roll]
  float _previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
  AccelerometerResults _acc_results;

  // Snaps and return the value of 50 if it is inside a range
  // that is considered to ease behavior (rounding errors, minor differences
  // from remote control and known range).
  // Otherwise returns given value.
  virtual int maybeSnapNeutral(int value)
  {
    // mark everything [45; 54] as 50
    if (value >= 45 && value <= 54)
    {
      return 50;
    }

    return value;
  }

  virtual byte getRelativeThrust(unsigned long rawThrust)
  {
    unsigned long mappedValue = map(rawThrust, 1025, 1874, PERCENT_LOW, PERCENT_TOP);

    return min(mappedValue, 100);
  }

  // This function extracts the value like this:
  // 0% -> backwards
  // 50% -> steady
  // 100% -> forwards
  virtual byte getRelativeMovementForwardBackward(unsigned long rawMFB)
  {
    unsigned long mappedValue = map(rawMFB, 1132, 1914, PERCENT_LOW, PERCENT_TOP);

    return min(maybeSnapNeutral(mappedValue), 100);
  }

  // This function extracts the value like this:
  // 0% -> left
  // 50% -> steady
  // 100% -> right
  virtual byte getRelativeMovementLeftRight(unsigned long rawMLR)
  {
    unsigned long mappedValue = map(rawMLR, 1128, 1887, PERCENT_LOW, PERCENT_TOP);

    return min(maybeSnapNeutral(mappedValue), 100);
  }

  // This function extracts the value like this:
  // 0% -> left
  // 50% -> steady
  // 100% -> right
  virtual byte getRotationLeftRightInPercent(unsigned long rawRLR)
  {
    unsigned long mappedValue = map(rawRLR, 1098, 1872, PERCENT_LOW, PERCENT_TOP);

    return min(maybeSnapNeutral(mappedValue), 100);
  }

public:
  FlightController(Servo servo1, Servo servo2, Servo servo3, Servo servo4)
  {
    if (
        !servo1.attached() ||
        !servo2.attached() ||
        !servo3.attached() ||
        !servo4.attached())
    {
      Serial.print("[EMERGENCY] [FlightController] At least one servo is not attached yet");
      Serial.println(", but FlightController expects attached ones!");
      this->_init_successful = false;
      return;
    }

    this->_servo_1 = servo1;
    this->_servo_2 = servo2;
    this->_servo_3 = servo3;
    this->_servo_4 = servo4;

    this->_error_sum[YAW] = 0;
    this->_error_sum[PITCH] = 0;
    this->_error_sum[ROLL] = 0;

    this->_previous_error[YAW] = 0;
    this->_previous_error[PITCH] = 0;
    this->_previous_error[ROLL] = 0;

    Serial.println("[INFO] [FlightController] Initalized.");
  }

  virtual bool initSuccessful()
  {
    return this->_init_successful;
  }

  virtual void UpdateState(RawValueSet raw_state, AccelerometerResults acc_results)
  {
    this->_relative_thrust = this->getRelativeThrust(raw_state.thrust);
    this->_relative_movement_f_b = this->getRelativeMovementForwardBackward(raw_state.movement_f_b);
    this->_relative_movement_l_r = this->getRelativeMovementLeftRight(raw_state.movement_l_r);
    this->_relative_rotation_l_r = this->getRotationLeftRightInPercent(raw_state.rotation_l_r);

    this->_acc_results = acc_results;
  }

  // Shuts down by telling the rotors to stop immediately
  virtual void Shutdown()
  {
    this->_servo_1.write(0);
    this->_servo_2.write(0);
    this->_servo_3.write(0);
    this->_servo_4.write(0);
  }

  // Writes the respective data to all rotors
  virtual void Commit()
  {
    Serial.println("Committing data to rotors …");

    float delta_err[3] = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
    float yaw_pid;
    float pitch_pid;
    float roll_pid;
    // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
    float errors[3];

    // calculate errors (i.e. differences between instructions and measurements)
    errors[YAW] = this->_relative_rotation_l_r - this->_acc_results.measures[YAW];
    errors[PITCH] = this->_relative_movement_f_b - this->_acc_results.measures[PITCH];
    errors[ROLL] = this->_relative_movement_l_r - this->_acc_results.measures[ROLL];

    // PID calculation
    // Calculate sum of _errors : Integral coefficients
    this->_error_sum[YAW] += errors[YAW];
    this->_error_sum[PITCH] += errors[PITCH];
    this->_error_sum[ROLL] += errors[ROLL];

    // Calculate error delta : Derivative coefficients
    delta_err[YAW] = errors[YAW] - this->_previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - this->_previous_error[PITCH];
    delta_err[ROLL] = errors[ROLL] - this->_previous_error[ROLL];

    // Save current error as previous_error for next time
    this->_previous_error[YAW] = errors[YAW];
    this->_previous_error[PITCH] = errors[PITCH];
    this->_previous_error[ROLL] = errors[ROLL];

    // PID = e*Kp + ∫e*Ki + Δe*Kd
    yaw_pid = (errors[YAW] * this->_Kp[YAW]) +
              (this->_error_sum[YAW] * this->_Ki[YAW]) +
              (delta_err[YAW] * this->_Kd[YAW]);
    pitch_pid = (errors[PITCH] * this->_Kp[PITCH]) +
                (this->_error_sum[PITCH] * this->_Ki[PITCH]) +
                (delta_err[PITCH] * this->_Kd[PITCH]);
    roll_pid = (errors[ROLL] * this->_Kp[ROLL]) +
               (this->_error_sum[ROLL] * this->_Ki[ROLL]) +
               (delta_err[ROLL] * this->_Kd[ROLL]);

    // Calculate pulse duration for each ESC
    this->_servo_1.write(
        minMax(this->_relative_thrust + roll_pid + pitch_pid - yaw_pid, ROTOR_1_MIN_STRENGTH, ROTOR_1_MAX_STRENGTH));
    this->_servo_2.write(
        minMax(this->_relative_thrust - roll_pid + pitch_pid + yaw_pid, ROTOR_2_MIN_STRENGTH, ROTOR_2_MAX_STRENGTH));
    this->_servo_3.write(
        minMax(this->_relative_thrust + roll_pid - pitch_pid + yaw_pid, ROTOR_3_MIN_STRENGTH, ROTOR_3_MAX_STRENGTH));
    this->_servo_4.write(
        minMax(this->_relative_thrust - roll_pid - pitch_pid - yaw_pid, ROTOR_4_MIN_STRENGTH, ROTOR_4_MAX_STRENGTH));
  }

  //Make sure that given value is not over min_value/max_value range.
  int minMax(int value, int min_value, int max_value)
  {
    if (value > max_value)
    {
      value = max_value;
    }
    else if (value < min_value)
    {
      value = min_value;
    }

    return value;
  }
};

// This class provides functions for checking whether we have enough electrical power,
// and avoid (permanently) harming the battery if it is going to be empty soon.
class PowerManager
{
private:
  float voltage_value_;
  unsigned long last_battery_check_ = 0;

public:
  // This gets called once this instance is created ("constructor")
  PowerManager(void)
  {
    // read voltage upon creation to avoid having an empty cache
    this->ReadVoltage();
  }

  // Reads and converts the raw value from the power supply and stores
  // it(can be recieved by `GetVoltage()`).
  virtual void ReadVoltage()
  {
    this->last_battery_check_ = millis();
    Serial.println("[INFO] [PowerManager] Reading new voltage value from power supply!");

    int rawValue = analogRead(CLEANUP_BATTERY_CHECK_PORT);

    this->voltage_value_ = rawValue * (5.00 / 1023.00) * 2;
#ifdef VERBOSE
    Serial.print("[INFO] [PowerManager] Battery voltage: ");
    Serial.print(this->voltage_value_);
    Serial.println("V");
#endif // VERBOSE
  }

  // Checks whether the value hold in cache is still up to date
  // (avoiding a persistent leak of time)
  virtual void MaybeUpdateCache()
  {
    // check whether the last read was (at least) CLEANUP_BATTERY_CHECK_INTERVAL seconds ago
    bool shouldUpdateCache = ((millis() - this->last_battery_check_) >= CLEANUP_BATTERY_CHECK_INTERVAL);

    if (shouldUpdateCache)
    {
      // update cache by reading the value and save it
      this->ReadVoltage();
    }
  }

  virtual float GetPowerSupplyVoltage()
  {
    // check whether we need to read the voltage again because of new conditions
    this->MaybeUpdateCache();

    return this->voltage_value_;
  }

  // Checks whether the power supply has enough power left over by checking whether
  // the voltage of it is high enough to further allow draining it.
  // Please check that each call
  virtual bool PowerSupplyIsOkay()
  {
    float voltageValue = this->GetPowerSupplyVoltage();

    // value is hardcoded to avoid an easy switch.
    // Any change here could damage the battery irrevocable and lead to fire.
    // By no means ignore this, Jens. Play safe. Be safe.
    // The value MUST include a safety buffer, just to play safe.
    if (voltageValue <= 7.40)
    {
      Serial.print("[EMERGENCY] [PowerManager] Power supply is too low, set to ");
      Serial.print(voltageValue);
      Serial.println("V");
      return false; // TODO: Make sure this is set to false in production!
    }

    return true;
  }
};

class RemoteControlManager
{
private:
  RawValueSet _values;

  // Reads and returns a raw PWM(pulse-width modulation) value from the given `channel_id` (i.e. the output port a cable
  // should be connected to).
  unsigned long _readValue(int channel_id)
  {
    return pulseIn(channel_id, HIGH, PULSE_IN_TIMEOUT);
  }

public:
  // Reads the values from the remote control receiver
  void Tick()
  {
    Serial.println("[INFO] [RemoteControlManager] Reading new values from Remote Control.");

    // horizontal (left/right)
    int channel1 = this->_readValue(CHANNEL_1_PORT);
    // horizontal movement (forward/backward)
    int channel2 = this->_readValue(CHANNEL_2_PORT);
    // vertical movement (thrust -> up/down)
    int channel3 = this->_readValue(CHANNEL_3_PORT);
    // rotation own axis
    int channel4 = this->_readValue(CHANNEL_4_PORT);

    RawValueSet values = {
        channel3,
        channel2,
        channel1,
        channel4};

    // TODO: Log values

    this->_values = values;
  }

  // Returns the last measured remote control values
  RawValueSet GetValues()
  {
    return this->_values;
  }
};

enum ProgramState
{
  STOPPED,
  STARTING,
  STARTED
};

ProgramState _program_state = ProgramState::STARTING;

class Main
{
private:
  FlightController *_flight_controller;
  PowerManager *_power_manager;
  RemoteControlManager *_remote_control_manager;
  AccelerationController *_acceleration_controller;

public:
  Main(
      FlightController *flight_controller,
      PowerManager *power_manager,
      RemoteControlManager *_remote_control_manager,
      AccelerationController *_acceleration_controller)
  {
    if (_program_state == ProgramState::STOPPED)
    {
      Serial.print("[EMERGENCY] [Main] An element set the program state to STOPPED for an unknown reason");
      Serial.println(" before the Main class took control!");
      return;
    }

    this->_flight_controller = flight_controller;
    this->_power_manager = power_manager;
    this->_remote_control_manager = _remote_control_manager;
    this->_acceleration_controller = _acceleration_controller;

    _program_state = ProgramState::STARTED;
  }

  void Tick()
  {
    if (_program_state == ProgramState::STOPPED)
    {
      // avoid busy loop by sleeping a little bit
      delay(3);
      return;
    }

    unsigned long tickStartTime = millis();

    if (!this->_power_manager->PowerSupplyIsOkay())
    {
#ifdef CLEANUP_BATTERY_CHECK
      this->Shutdown();
#endif
    }

    this->_acceleration_controller->Tick();

    this->_remote_control_manager->Tick();

    this->_flight_controller->UpdateState(
        this->_remote_control_manager->GetValues(), this->_acceleration_controller->GetResults());

    this->_flight_controller->Commit();

    // tick_duration is lower than ROUGH_TICK_TIME now, thus recalculate it
    unsigned long tickDuration = millis() - tickStartTime;

    Serial.print("[INFO] [Main] Tick took ");
    Serial.print(tickDuration);
    Serial.println("ms");

    // check whether the cleanup task took too long
    if (tickDuration >= ROUGH_TICK_TIME)
    {
      Serial.println("[WARNING] Tick took too much time!");
      Serial.println();
      return;
    }

    // calculate how much time we need to sleep now
    unsigned long sleepTime = ROUGH_TICK_TIME - (millis() - tickStartTime);

    // ignore overflows/underflows
    if (sleepTime <= 0 || sleepTime >= 1000)
    {
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

  // Marks this program as shut down, shuts down the servo.
  // Please note that does *NOT* actually terminate the program
  // (as it is impossible to turn an Arduino off by itself)!
  void Shutdown()
  {
    // mark program as shut down
    _program_state = ProgramState::STOPPED;

    this->_flight_controller->Shutdown();

    Serial.println("[EMERGENCY] [Main] Shutdown!");
  }
};

Main *_main;

// This is the entry point for our program.
// This method is being called by the bootloader after its usual boot-up stuff
// and our chance to initalize some basic stuff(like input ports, rotors…)
void setup()
{
  // should be close (if not equal) to zero, otherwise the bootloader is doing quite crazy stuff?
  start_time = millis();

  // Initalize input pins for reading values from them
  for (int i = 0; i < kInputPortLength; i++)
  {
    pinMode(INPUT_PORTS[i], INPUT);
  }

  // Initalize output pins for sending values to them
  for (int i = 0; i < kOutputPortLength; i++)
  {
    pinMode(OUTPUT_PORTS[i], OUTPUT);
  }

  // seed the (pseudo-)random number generator asap on an **unconnected** pin
  randomSeed(analogRead(RANDOM_SEED_PORT));

  // Connect to a (potential) computer for outputting debug information on baud 9600
  Serial.begin(1e6);
  while (!Serial)
    ;

  Serial.print("[INFO] [setup()] Program starting … Start time was ");
  Serial.print(start_time);
  Serial.println("ms");

  // Init PowerManager and execute an immediate scan whether we are alright to fly
  // in the sky!
  PowerManager *power_mgr = new PowerManager();
#ifdef CLEANUP_BATTERY_CHECK
  if (!power_mgr->PowerSupplyIsOkay())
  {
    Serial.println("[EMERGENCY] [setup()] Power supply too low!");
    _program_state = ProgramState::STOPPED;
  }
#endif // CLEANUP_BATTERY_CHECK

  // Set output pins
  rotor_1.attach(ROTOR_1_PORT, ROTOR_1_MIN_STRENGTH, ROTOR_1_MAX_STRENGTH);
  rotor_2.attach(ROTOR_2_PORT, ROTOR_2_MIN_STRENGTH, ROTOR_2_MAX_STRENGTH);
  rotor_3.attach(ROTOR_3_PORT, ROTOR_3_MIN_STRENGTH, ROTOR_3_MAX_STRENGTH);
  rotor_4.attach(ROTOR_4_PORT, ROTOR_4_MIN_STRENGTH, ROTOR_4_MAX_STRENGTH);

  FlightController *flight_controller = new FlightController(rotor_1, rotor_2, rotor_3, rotor_4);
  if (!flight_controller->initSuccessful())
  {
    _program_state = ProgramState::STOPPED;
  }

  RemoteControlManager *rc_manager = new RemoteControlManager();

  AccelerationController *acc_manager = new AccelerationController();

  _main = new Main(flight_controller, power_mgr, rc_manager, acc_manager);

  Serial.print("Finished boot process. Startup took ");
  Serial.print(millis() - start_time);
  Serial.println("ms");

  // arduino bootloader now starts with calling loop() over and over again as soon as we return
}

// Helper utility function to avoid clutter-code in the main function
// that logs:
//  * a) the name this value stands for(the first character should be upper-cased),
//  * b) the value itself and
//  * c) the raw value (we fed our algorithms with)
// so they are printed out on the Serial Monitor(iff connected).
void logValue(unsigned long value, String name, unsigned long rawValue)
{
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

// Returns how long the arduino is running in milliseconds, useful for debug purposes.
unsigned long getRanTime()
{
  return millis() - start_time;
}

// Called by the arduino bootloader after calling setup() over and over again, so
// useful for us to implement ticks
void loop()
{
  _main->Tick();
}
