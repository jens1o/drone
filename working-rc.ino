#include<Servo.h>
#include<Wire.h>
#include<I2Cdev.h>
#include<MPU6050_6Axis_MotionApps20.h>

// Defines whether there is verbose logging on the Serial Monitor
// commented out = production
// not commented out = debug mode
#define VERBOSE

// #define CLEANUP_BATTERY_CHECK
#define CLEANUP_BATTERY_CHECK_PORT A0
#define CLEANUP_BATTERY_CHECK_INTERVAL (10 * 1000)

#define PERCENT_TOP 100
#define PERCENT_LOW 0
// MUST NOT be too less, otherwise we will get weird values
#define PULSE_IN_TIMEOUT 25000
// time a tick should roughly take.
#define ROUGH_TICK_TIME 225 // 225

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

// The delta between the *_MIN_STRENGTH and *_MAX_STRENGTH should be 1856(as given by the Servo doc).

#define ROTOR_1_MIN_STRENGTH 995 // (60)
#define ROTOR_1_MAX_STRENGTH 2945
#define ROTOR_1_DIRECTION RIGHT

#define ROTOR_2_MIN_STRENGTH 1025 // (60)
#define ROTOR_2_MAX_STRENGTH 3075
#define ROTOR_2_DIRECTION LEFT

#define ROTOR_3_MIN_STRENGTH 995 // (60)
#define ROTOR_3_MAX_STRENGTH 2856
#define ROTOR_3_DIRECTION RIGHT

#define ROTOR_4_MIN_STRENGTH 975 // (60)
#define ROTOR_4_MAX_STRENGTH 2716
#define ROTOR_4_DIRECTION LEFT

#define FREQ 250   // Sampling frequency
#define SSF_GYRO 65.5  // Sensitivity Scale Factor of the gyrometer from the datasheet
#define GYRO_INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#define X 0     // X axis
#define Y 1     // Y axis
#define Z 2     // Z axis

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
  float acc_angle[3] = {0, 0, 0};
  int temperature;
  /**
     Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
      - Left wing up implies a positive roll
      - Nose up implies a positive pitch
      - Nose right implies a positive yaw
  */
  float measures[3] = {0, 0, 0};
  // Calculated angles from gyro's values in that order: X, Y, Z
  float gyro_angle[3]  = {0, 0, 0};
  // Total 3D acceleration vector in m/s²
  long acc_total_vector;
};

unsigned long start_time;

int last_thrust_value = 0;

int const INPUT_PORTS[] = {RANDOM_SEED_PORT, CHANNEL_1_PORT, CHANNEL_2_PORT, CHANNEL_3_PORT, CHANNEL_4_PORT};
// in the arduino world, there is no reliable way of knowing how long an array is, thus computing this at compile time
#define kInputPortLength (sizeof(INPUT_PORTS)/sizeof(int))

int const OUTPUT_PORTS[] = {ROTOR_1_PORT, ROTOR_2_PORT, ROTOR_3_PORT, ROTOR_4_PORT};
// in the arduino world, there is no reliable way of knowing how long an array is, thus computing this at compile time
#define kOutputPortLength (sizeof(OUTPUT_PORTS)/sizeof(int))

Servo rotor_1, rotor_2, rotor_3, rotor_4;

const int MPU_ADDRESS = 0x68;  // I2C address of the MPU-6050
// DO NOT USE this variable outside of AccelerationController, it must be placed in the global scope as the Arduino programming language(Processing) is
// unable to process static class variables …
// do not leave out the `volatile`, as it is needed here(although heavily decreasing the abilities of the
// optimizer)
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
    uint16_t _fifoCount;
    uint8_t _mpuInterruptPacketSize;

    static void _hasDataReadyCallback() {
      __acc_controller_has_new_data = true;
    }

  public:
    AccelerationController(void) {
      unsigned long initStartTime = millis();

      // start listening
      Wire.begin();
      // set I2C clock to 400 kHz
      Wire.setClock(400000);

      this->_mpu.initialize();

      pinMode(GYRO_INTERRUPT_PIN, INPUT);

      // Test connection to verify it can work
      if (!this->_mpu.testConnection()) {
        _init_successful = false;
        Serial.println("[EMERGENCY] [AccelerationController] MPU-6050 connection failed!");
        return;
      }

      delay(200);

      uint8_t deviceStatus = this->_mpu.dmpInitialize();

      // calibrated using a compass and try and error
      this->_mpu.setXGyroOffset(220);
      this->_mpu.setYGyroOffset(76);
      this->_mpu.setZGyroOffset(-85);
      this->_mpu.setZAccelOffset(1788);

      if (deviceStatus != 0) {
        _init_successful = false;
        Serial.println("[WARNING] [AccelerationController] Non-zero error code when initalizing MPU!");
        return;
      }

      this->_mpu.setDMPEnabled(true);

      digitalPinToInterrupt(GYRO_INTERRUPT_PIN);

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

    bool initSuccessful() {
      return this->_init_successful;
    }

    void loop() {
      // I miss you, exceptions.
      if (!this->initSuccessful()) return;

      // wait for MPU interrupt or extra packet(s) available
      //while (!__acc_controller_has_new_data && fifoCount < packetSize) {
        //if (__acc_controller_has_new_data && fifoCount < packetSize) {
          // try to get out of the infinite loop
          //fifoCount = mpu.getFIFOCount();
        //}
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
      //}
      // reset to ready
     //__acc_controller_has_new_data = false;
    }
};

class FlightController
{
  private:
    byte _relative_1 = 0;
    byte _relative_2 = 0;
    byte _relative_3 = 0;
    byte _relative_4 = 0;
    byte _relative_thrust = 0;
    byte _relative_movement_f_b = 0;
    byte _relative_movement_l_r = 0;
    byte _relative_rotation_l_r = 0;
    bool _init_successful = true;
    Servo _servo_1;
    Servo _servo_2;
    Servo _servo_3;
    Servo _servo_4;

    // Snaps and return the value of 50 if it is inside a range
    // that is considered to ease behavior (rounding errors, minor differences
    // from remote control and known range).
    // Otherwise returns given value.
    virtual int maybeSnapNeutral(int value) {
      // mark everything 45..=54 as 50
      if (value >= 45 && value <= 54) {
        return 50;
      }

      return value;
    }

    virtual byte getRelativeThrust(unsigned long rawThrust) {
      unsigned long mappedValue = map(rawThrust, 1025, 1874, PERCENT_LOW, PERCENT_TOP);

      return min(mappedValue, 100);
    }

    // This function extracts the value like this:
    // 0% -> backwards
    // 50% -> steady
    // 100% -> forwards
    virtual byte getRelativeMovementForwardBackward(unsigned long rawMFB) {
      unsigned long mappedValue = map(rawMFB, 1132, 1914, PERCENT_LOW, PERCENT_TOP);

      return min(maybeSnapNeutral(mappedValue), 100);
    }

    // This function extracts the value like this:
    // 0% -> left
    // 50% -> steady
    // 100% -> right
    virtual byte getRelativeMovementLeftRight(unsigned long rawMLR) {
      unsigned long mappedValue = map(rawMLR, 1128, 1887, PERCENT_LOW, PERCENT_TOP);

      return min(maybeSnapNeutral(mappedValue), 100);
    }


    // This function extracts the value like this:
    // 0% -> left
    // 50% -> steady
    // 100% -> right
    virtual byte getRotationLeftRightInPercent(unsigned long rawRLR) {
      unsigned long mappedValue = map(rawRLR, 1098, 1872, PERCENT_LOW, PERCENT_TOP);

      return min(maybeSnapNeutral(mappedValue), 100);
    }

  public:
    FlightController(Servo servo1, Servo servo2, Servo servo3, Servo servo4) {
      if
      (
        !servo1.attached() ||
        !servo2.attached() ||
        !servo3.attached() ||
        !servo4.attached()
      )
      {
        Serial.println("[EMERGENCY] [FlightController] At least one servo is not attached yet, but FlightController expects attached ones!");
        this->_init_successful = false;
        return;
      }

      this->_servo_1 = servo1;
      this->_servo_2 = servo2;
      this->_servo_3 = servo3;
      this->_servo_4 = servo4;

      Serial.println("[INFO] [FlightController] Initalized.");
    }

    virtual bool initSuccessful() {
      return this->_init_successful;
    }

    virtual void UpdateState(RawValueSet raw_state/*, AccelerometerResults acc_results*/) {
      this->_relative_thrust = this->getRelativeThrust(raw_state.thrust);
      this->_relative_movement_f_b = this->getRelativeMovementForwardBackward(raw_state.movement_f_b);
      this->_relative_movement_l_r = this->getRelativeMovementLeftRight(raw_state.movement_l_r);
      this->_relative_rotation_l_r = this->getRotationLeftRightInPercent(raw_state.rotation_l_r);

    }

    virtual void Shutdown() {
      this->_servo_1.write(0);
      this->_servo_2.write(0);
      this->_servo_3.write(0);
      this->_servo_4.write(0);
    }

    // Writes the respective data to all rotors
    virtual void Commit() {
      // TODO
      Serial.print("Writing data to rotors. Thrust: ");
      Serial.println(this->_relative_thrust);
      this->_servo_1.write(this->_relative_thrust);
      this->_servo_2.write(this->_relative_thrust);
      this->_servo_3.write(this->_relative_thrust);
      this->_servo_4.write(this->_relative_thrust);
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
    // This gets called once this instance is created
    PowerManager(void) {
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

      if (shouldUpdateCache) {
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
      if (voltageValue <= 7.40) {
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
    unsigned long _readValue(int channel_id) {
      return pulseIn(channel_id, HIGH, PULSE_IN_TIMEOUT);
    }

  public:
    // Reads the values
    void Update() {
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
        channel4
      };

      // TODO: Log values
      Serial.print("Thrust value: ");
      Serial.println(values.thrust);

      this->_values = values;
    }

    // Returns the last measured remote control values
    RawValueSet GetValues() {
      return this->_values;
    }
};

enum ProgramState {
  STOPPED,
  STARTING,
  STARTED
};

ProgramState _program_state = ProgramState::STARTING;

class Main {
  private:
    FlightController* _flight_controller;
    PowerManager* _power_manager;
    RemoteControlManager* _remote_control_manager;
    AccelerationController* _acceleration_controller;

  public:
    Main(FlightController* flight_controller, PowerManager* power_manager, RemoteControlManager* _remote_control_manager, AccelerationController* _acceleration_controller) {
      if (_program_state == ProgramState::STOPPED) {
        Serial.println("[EMERGENCY] [Main] An element set the program state to STOPPED for an unknown reason before the Main class took control!");
        return;
      }

      this->_flight_controller = flight_controller;
      this->_power_manager = power_manager;
      this->_remote_control_manager = _remote_control_manager;
      this->_acceleration_controller = _acceleration_controller;
    }

    void Tick() {
      if (_program_state == ProgramState::STOPPED) {
        // avoid busy loop by sleeping a little bit
        delay(3);
        return;
      }

      unsigned long tickStartTime = millis();

      if (!this->_power_manager->PowerSupplyIsOkay()) {
#ifdef CLEANUP_BATTERY_CHECK
        this->Shutdown();
#endif
      }

      this->_remote_control_manager->Update();

      this->_flight_controller->UpdateState(this->_remote_control_manager->GetValues()/*, this->_acceleration_controller->GetSensorResults()*/);

      this->_flight_controller->Commit();

      // tick_duration is lower than ROUGH_TICK_TIME now, thus recalculate it
      unsigned long tickDuration = millis() - tickStartTime;

      Serial.print("[INFO] [Main] Tick took ");
      Serial.print(tickDuration);
      Serial.println("ms");

      // check whether the cleanup task took too long
      if (tickDuration >= ROUGH_TICK_TIME) {
        Serial.println("[WARNING] Tick took too much time!");
        Serial.println();
        return;
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

    // Marks this program as shut down, shuts down the servo.
    // Please note that does *NOT* actually terminate the program
    // (as it is impossible to turn an Arduino off by itself)!
    void Shutdown() {
      // mark program as shut down
      _program_state = ProgramState::STOPPED;

      this->_flight_controller->Shutdown();

      Serial.println("[EMERGENCY] [Main] Shutdown!");
    }
};


Main* _main;

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
  Serial.begin(1e6);
  while (!Serial);

  // Init PowerManager and execute an immediate scan whether we are alright to fly
  // in the sky!
  PowerManager* power_mgr = new PowerManager();
#ifdef CLEANUP_BATTERY_CHECK
  if (!power_mgr->PowerSupplyIsOkay()) {
    Serial.println("[EMERGENCY] [setup()] Power supply too low!");
    _program_state = ProgramState::STOPPED;
  }
#endif // CLEANUP_BATTERY_CHECK

  // Set output pins
  rotor_1.attach(ROTOR_1_PORT, ROTOR_1_MIN_STRENGTH, ROTOR_1_MAX_STRENGTH);
  rotor_2.attach(ROTOR_2_PORT, ROTOR_2_MIN_STRENGTH, ROTOR_2_MAX_STRENGTH);
  rotor_3.attach(ROTOR_3_PORT, ROTOR_3_MIN_STRENGTH, ROTOR_3_MAX_STRENGTH);
  rotor_4.attach(ROTOR_4_PORT, ROTOR_4_MIN_STRENGTH, ROTOR_4_MAX_STRENGTH);

  FlightController* flight_controller = new FlightController(rotor_1, rotor_2, rotor_3, rotor_4);
  if (!flight_controller->initSuccessful()) {
    _program_state = ProgramState::STOPPED;
  }

  RemoteControlManager* rc_manager = new RemoteControlManager();

  AccelerationController* acc_manager = new AccelerationController();

  _main = new Main(flight_controller, power_mgr, rc_manager, acc_manager);

  Serial.print("Finish boot process.. Startup took ");
  Serial.print(millis() - start_time);
  Serial.println("ms");

  // arduino bootloader now starts with calling loop() over and over again as soon as we return
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

void loop() {
  _main->Tick();
}
