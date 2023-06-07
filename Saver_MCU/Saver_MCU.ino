#define TRUE            1
#define FALSE           0
#define SIG_POWER_MAX   -140 
#define SIG_POWER_MIN   -600
#define P_K             1 // delete
#define DEAD_AHEAD      180
#define ANGLE_WIDE      45
#define ANGLE_TIGHT     15

#define PORT_PIN      D10
#define STAR_PIN      D9
#define PI_PIN        A5
#define KRAKEN_PIN    A4
#define E_STOP_PIN    D6
#define MOTOR_PIN     PB15  //COPI pin
//#define OFF_PIN       D11
//#define START_PIN     D12
#define SWITCH_PIN    D12
#define SWITCH_ON     0
#define SWITCH_OFF    1

#define MSG_BUFF_SIZE 24
#define MAX_MSG_LEN (MSG_BUFF_SIZE - 1)

#define THRUSTER_MAX_REVERSE 1000 // microsec
#define THRUSTER_NEUTRAL 1500 // microsec
#define THRUSTER_MAX_FORWARD 2000 // microsec
#define THRUSTER_PWM_PERIOD 4000 //microsec
#define THRUSTER_RANGE (THRUSTER_MAX_FORWARD - THRUSTER_NEUTRAL)
#define THRUSTER_UPDATE_MAX_INTERVAL 2 // microsec. 1 breaks it
#define THRUSTER_UPDATE_TIME 7500 // microsec

#define STALE_DATA_TIMEOUT 2000000 //microsec

typedef enum STATE_TYPE
{
  GO,
  STOP,
}STATE_TYPE;

typedef enum AUTO_STATE
{
  ASSESS,
  CW,
  CCW,
  FORWARD
}AUTO_STATE;

//stuff needed for timers to do PWM
TIM_TypeDef *tim1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PORT_PIN), PinMap_PWM);  //TIM4 channel 3
TIM_TypeDef *tim2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(STAR_PIN), PinMap_PWM);  //TIM4 channel 4 (or at least it should be)
HardwareTimer *port_timer = new HardwareTimer(tim1);  //TIM4 channel 3
HardwareTimer *star_timer = new HardwareTimer(tim2);  //TIM4 channel 4
uint32_t port_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PORT_PIN), PinMap_PWM));
uint32_t star_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(STAR_PIN), PinMap_PWM));

TIM_TypeDef *tim3 = TIM3;
HardwareTimer *update_thrusters_timer = new HardwareTimer(tim3);

TIM_TypeDef *tim5 = TIM5;
HardwareTimer *stale_data_timer = new HardwareTimer(tim5);

int32_t PortTarget = THRUSTER_NEUTRAL;
int32_t StarTarget = THRUSTER_NEUTRAL;
int32_t NextPortValue = THRUSTER_NEUTRAL;
int32_t NextStarValue = THRUSTER_NEUTRAL;
uint8_t CalculateNext = FALSE;

//attach serial to D0 (Rx) and D1 (tx)
HardwareSerial PiSerial(D0, D1);

//starting state for the state machine
//didn't want to make it global but arduino is stupid
STATE_TYPE state = STOP;

int standby_flag = 0;
int start_flag = 0;

// System Configuration Variables
static int SysForwardPowerPercentage = 40;
static int SysReversePowerPercentage = 40;
static int SysForwardPowerLimit = THRUSTER_NEUTRAL + THRUSTER_RANGE * SysForwardPowerPercentage / 100;
static int SysReversePowerLimit = THRUSTER_NEUTRAL - THRUSTER_RANGE * SysReversePowerPercentage / 100;
static int SysPivotFWValue = 1512;
static int SysPivotRVValue = 1500;
static int SysThrusterUpdateTime = THRUSTER_UPDATE_TIME;
static int SysThrusterUpdateInterval = THRUSTER_UPDATE_MAX_INTERVAL;
static int SysStaleDataTimeout = STALE_DATA_TIMEOUT;
static int SysSigPowerMax = SIG_POWER_MAX;
static int SysSigPowerMin = SIG_POWER_MIN;
static int SysAngleWide = ANGLE_WIDE;
static int SysAngleTight = ANGLE_TIGHT;
static int SysForwardOffset = 40;
static int SysForwardOffsetFar = 40;
static int SysForwardOffsetNear = 40;
static int SysOrder = 4;
static int SysReflect = 1;
static int SysDoaOffset = 0;
static int SysSpeedChangePower = -300;

// Swappable function pointers
static int32_t (*process_data)(int32_t, int32_t, int32_t, uint8_t); // preprocessing on data
static void (*move_boat)(int32_t); // movement control algorithm

///////////////////////////////////////////////////////////////////////////////
///                          Movement Modes
///////////////////////////////////////////////////////////////////////////////

void auto_stop(int32_t doa)
{
  // Don't set target thruster values. Would override manual.
  Serial.printf("STOP:\tPort=%i   Star=%i\n", PortTarget, StarTarget);
}

// void pivot_cw(int32_t doa)
// {
//   if (doa > DEAD_AHEAD - SysAngleTight && doa < DEAD_AHEAD + SysAngleTight)
//   {
//     PortTarget = THRUSTER_NEUTRAL;
//     StarTarget = THRUSTER_NEUTRAL;
//     move_boat = auto_forward;
//   }
//   else
//   {
//     PortTarget = SysPivotFWValue;
//     StarTarget = SysPivotRVValue;
//   }
//   Serial.print("Pivot CW => ");
// }

// void pivot_ccw(int32_t doa)
// {
//   if (doa > DEAD_AHEAD - SysAngleTight && doa < DEAD_AHEAD + SysAngleTight)
//   {
//     PortTarget = THRUSTER_NEUTRAL;
//     StarTarget = THRUSTER_NEUTRAL;
//     move_boat = auto_forward;
//   }
//   else
//   {
//     PortTarget = SysPivotRVValue;
//     StarTarget = SysPivotFWValue;
//   }
//   Serial.print("Pivot CCW => ");
// }

void move_auto(int32_t doa)
{
  static AUTO_STATE mode = ASSESS;

  switch (mode)
  {
    case ASSESS:
      if (doa >= 0 && doa < DEAD_AHEAD - SysAngleTight)
      {
        mode = CW;
        Serial.println("Assess: Auto mode to CW");
      }
      else if (doa > DEAD_AHEAD + SysAngleTight && doa < 360)
      {
        mode = CCW;
        Serial.println("Assess: Auto mode to CCW");
      }
      else
      {
        mode = FORWARD;
        Serial.println("Assess: Auto mode to FORWARD");
      }
    break;

    case FORWARD:
      if (doa >= 0 && doa < DEAD_AHEAD - SysAngleWide)
      {
        mode = CW;
        Serial.println("Auto mode to CW");
      }
      if (doa > DEAD_AHEAD + SysAngleWide && doa < 360)
      {
        mode = CCW;
        Serial.println("Auto mode to CCW");
      }
    break;

    case CW: // same region as CCW
      if (doa > DEAD_AHEAD - SysAngleTight && doa < DEAD_AHEAD + SysAngleTight)
      {
        mode = FORWARD;
        Serial.println("Auto mode to FORWARD");
      }
    break;

    case CCW: // same region as CW
      if (doa > DEAD_AHEAD - SysAngleTight && doa < DEAD_AHEAD + SysAngleTight)
      {
        mode = FORWARD;
        Serial.println("Auto mode to FORWARD");
      }
    break;

    default:
      Serial.println("Error: invalid auto mode state 1");
    break;
  }

  switch(mode)
  {
    case FORWARD:
      int32_t offset;
      offset = THRUSTER_RANGE * pow((abs((double)doa - DEAD_AHEAD)) / DEAD_AHEAD, SysOrder);
      
      if (doa >= DEAD_AHEAD)
      {
        // may have these backwards
        PortTarget = THRUSTER_NEUTRAL + SysForwardOffset + offset;
        StarTarget = THRUSTER_NEUTRAL + SysForwardOffset - offset;
      }
      else
      {
        PortTarget = THRUSTER_NEUTRAL + SysForwardOffset - offset;
        StarTarget = THRUSTER_NEUTRAL + SysForwardOffset + offset;
      }
      Serial.printf("FORWARD:\tPort=%i  Star=%i\n", PortTarget, StarTarget);
    break;

    case CW:
      PortTarget = SysPivotFWValue;
      StarTarget = SysPivotRVValue;
      Serial.printf("CW:\tPort=%i  Star=%i\n", PortTarget, StarTarget);
    break;

    case CCW:
      PortTarget = SysPivotRVValue;
      StarTarget = SysPivotFWValue;
      Serial.printf("CW:\tPort=%i  Star=%i\n", PortTarget, StarTarget);
    break;

    case ASSESS: // shouldn't happen
      PortTarget = THRUSTER_NEUTRAL;
      StarTarget = THRUSTER_NEUTRAL;
    break;

    default:
      Serial.println("Error: invalid auto mode state 2");
    break;
  }
}

// void auto_forward(int32_t doa)
// {

//   int32_t offset;

//   if (doa < DEAD_AHEAD - SysAngleWide)
//   {
//     PortTarget = THRUSTER_NEUTRAL;
//     StarTarget = THRUSTER_NEUTRAL;
//     move_boat = pivot_cw;
//     return;
//   }
//   if (doa > DEAD_AHEAD + SysAngleWide)
//   {
//     PortTarget = THRUSTER_NEUTRAL;
//     StarTarget = THRUSTER_NEUTRAL;
//     move_boat = pivot_cw;
//   }

//   offset = THRUSTER_RANGE * pow((abs((double)doa - DEAD_AHEAD)) / DEAD_AHEAD, SysOrder);

//   if (doa >= DEAD_AHEAD)
//   {
//     // may have these backwards
//     PortTarget = THRUSTER_NEUTRAL + SysForwardOffset + offset;
//     StarTarget = THRUSTER_NEUTRAL + SysForwardOffset - offset;
//   }
//   else
//   {
//     PortTarget = THRUSTER_NEUTRAL + SysForwardOffset - offset;
//     StarTarget = THRUSTER_NEUTRAL + SysForwardOffset + offset;
//   }
//   Serial.print("Move forward => ");
// }

///////////////////////////////////////////////////////////////////////////////
///                      Preprocessing Functions
///////////////////////////////////////////////////////////////////////////////
int32_t single_doa(int32_t doa, int32_t conf, int32_t power, uint8_t reset)
{
  int result = abs(360*SysReflect - ((doa + SysDoaOffset) % 360));
  Serial.printf("res=%i, doa=%i, offset=%i, reflect=%i", result, doa, SysDoaOffset, SysReflect);
  return result;
}

int32_t window_doa(int32_t doa, int32_t conf, int32_t power, uint8_t reset)
{
  static const uint8_t window_size = 3;
  static int32_t window[window_size] = {DEAD_AHEAD};
  static uint8_t num_samples = 0;
  static uint8_t filled_window = FALSE;

  // reset when changing preprocessing method
  if (reset)
  {
    for (int i = 0; i < window_size; i++)
    {
      window[i] = DEAD_AHEAD;
    }
    filled_window = FALSE;
    return -1; // return value should indicate invalid doa
  }

  // add new entry
  window[num_samples++ % window_size] = doa;

  if (num_samples >= window_size)
  {
    filled_window = TRUE;
  }

  if (!filled_window)
  {
    return -1; 
  }

  int32_t sum = 0;
  for(int i = 0; i < window_size; i++)
  {
    sum += window[i];
  }

  int result = abs(360*SysReflect - (((sum / window_size) + SysDoaOffset) % 360));

  Serial.printf("res=%i, doa=%i, pow=%i, offset=%i, reflect=%i\n", result, doa, power, SysDoaOffset, SysReflect);

  return result;
}

static int32_t (*preprocessors[2])(int32_t, int32_t, int32_t, uint8_t) = 
{
  single_doa,
  window_doa
};

// void (*move_modes[5])(int32_t) =
// {
//   pivot_cw,
//   pivot_ccw,
//   move_auto,
//   move_manual  
// };

void setup() {
  Serial.begin(115200); // PC
  PiSerial.begin(115200); // Pi

  // Relays
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(PI_PIN, OUTPUT);
  pinMode(KRAKEN_PIN, OUTPUT);

  delay(2000);

  // Pin Interrupts
  pinMode(E_STOP_PIN,INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  // pinMode(OFF_PIN, INPUT_PULLUP);
  // pinMode(START_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), E_stop, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), Check_Switch, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(OFF_PIN), Turn_Off, FALLING);
  // attachInterrupt(digitalPinToInterrupt(START_PIN), Turn_On, FALLING);

  // Thruster PWMs  
  port_timer->pause();
  star_timer->pause();  
  port_timer->setMode(port_channel, TIMER_OUTPUT_COMPARE_PWM1, PORT_PIN);
  star_timer->setMode(star_channel, TIMER_OUTPUT_COMPARE_PWM1, STAR_PIN);
  port_timer->setOverflow(THRUSTER_PWM_PERIOD, MICROSEC_FORMAT);
  star_timer->setOverflow(THRUSTER_PWM_PERIOD, MICROSEC_FORMAT);
  port_timer->setCaptureCompare(port_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);

  // Thruster update timer
  update_thrusters_timer->pause();
  update_thrusters_timer->setOverflow(THRUSTER_UPDATE_TIME, MICROSEC_FORMAT);
  update_thrusters_timer->attachInterrupt(update_thrusters);

  stale_data_timer->pause();
  stale_data_timer->setOverflow(STALE_DATA_TIMEOUT, MICROSEC_FORMAT);
  stale_data_timer->attachInterrupt(stale_data);
  
  process_data = single_doa;
  move_boat = move_auto;

  Serial.println("Setup Complete");

}

void loop() {
  
  //TODO:check to see if this will cause issues. Might change them to static
  static char PiMsg[MSG_BUFF_SIZE] = {0};
  static char PcMsg[MSG_BUFF_SIZE] = {0};
  static uint8_t PiMsgLen = 0;
  static uint8_t PcMsgLen = 0;
  static uint8_t PiMsgReady = FALSE;
  static uint8_t PcMsgReady = FALSE;
  static uint8_t NewPiData = FALSE;

  static int32_t power = 0; 
  static int32_t doa = 0;
  static int32_t conf = 0;  
  static int32_t processed_doa = DEAD_AHEAD;
  static uint8_t filled_window;

  int status = 0;

  NewPiData = FALSE;
  checkPiSerial(PiMsg, &PiMsgLen, &PiMsgReady);
  if (PiMsgReady)
  {
    status = sscanf(PiMsg, "%i,%i,%i", &doa, &conf, &power);
    PiMsgReady = FALSE;
    PiMsgLen = 0;

    if (status == 3 && doa >= 0 && doa < 360)
    {
      NewPiData = TRUE;
    }
    if (status > 0 && doa >= 400)
    {
      config_system(doa, conf, power, status);
    }
  }

  if (CalculateNext) {
    next_thruster_values();
    CalculateNext = FALSE;    
  }

  switch(state)
  {
    case GO:

      if (!NewPiData)
      {
        break;
      }


      if(power > SysSigPowerMax) // close to target
      {
        Serial.println("Stong signal. Stopping");
        state = STOP;
        break;
      }
      else if(power < SysSigPowerMin) // beacon off
      {
        Serial.println("Weak signal. Ignoring");
        break;
      }
      else // new data to process
      {
        stale_data_timer->pause();
        stale_data_timer->setCount(1);
        stale_data_timer->resume();

        if (power > SysSpeedChangePower)
        {
          SysForwardOffset = SysForwardOffsetNear;
        }
        else
        {
          SysForwardOffset = SysForwardOffsetFar;
        }

        processed_doa = process_data(doa, conf, power, FALSE);
        if (processed_doa >= 0 && processed_doa < 360)
        {
          move_boat(processed_doa);
        }
      }
    break;

    case STOP: // Separate stop mode for stale data?
      StarTarget = THRUSTER_NEUTRAL;
      PortTarget = THRUSTER_NEUTRAL;
      if (NewPiData && power < SysSigPowerMax)
      {
        Serial.println("Out of target range. Moving");
        state = GO;
      }
    break;

    default:
    break; 
  } 
}

///////////////////////////////////////////////////////////////////////////////
///                       Configure System                                  ///
///////////////////////////////////////////////////////////////////////////////
void config_system(int32_t opcode, int32_t arg1, int32_t arg2, int status)
{
  switch (status)
  {
    case 1: // no arg1 or arg2
      switch(opcode)
      {
        case 400: // stop
          stale_data_timer->pause();
          stale_data_timer->setCount(1);
          PortTarget = THRUSTER_NEUTRAL;
          StarTarget = THRUSTER_NEUTRAL;
          move_boat = auto_stop;
          Serial.println("Move Mode: Stop");
        break;

        case 401: // autonomous
          move_boat = move_auto;
          stale_data_timer->resume();
          Serial.println("Move Mode: Auto");
        break;

        case 402: // pivot cw
          stale_data_timer->pause();
          stale_data_timer->setCount(1);
          PortTarget = SysPivotFWValue;
          StarTarget = SysPivotRVValue;
          move_boat = auto_stop;
          Serial.println("Move Mode: Manual CW");
        break;

        case 403: // pivot ccw
          stale_data_timer->pause();
          stale_data_timer->setCount(1);
          PortTarget = SysPivotRVValue;
          StarTarget = SysPivotFWValue;
          move_boat = auto_stop;
          Serial.println("Move Mode: Manual CCW");
        break;

        case 404: // forward
          PortTarget = THRUSTER_NEUTRAL + SysForwardOffset;
          StarTarget = THRUSTER_NEUTRAL + SysForwardOffset;
          stale_data_timer->pause();
          stale_data_timer->setCount(1);
          move_boat = auto_stop;
          Serial.println("Move Mode: Manual FORWARD");
        break;

        case 410:
          process_data = single_doa;
          Serial.println("Process Mode: Single");
        break;

        case 411:
          process_data = window_doa;
          Serial.println("Process Mode: Window");
        break;
        default:
        break;
      }
    break;
    case 2: // arg1
      switch(opcode)
      {
        case 500:
          if (arg1 >= 0 && arg1 <= 100)
          {
            SysForwardPowerPercentage = arg1;
            SysForwardPowerLimit = THRUSTER_NEUTRAL + THRUSTER_RANGE * SysForwardPowerPercentage / 100;
            Serial.printf("Forward thruster limit to %i%%\n",arg1); 
          }
        break;

        case 501:
          if (arg1 >= 0 && arg1 <= 100)
          {
            SysReversePowerPercentage = arg1;
            SysReversePowerLimit = THRUSTER_NEUTRAL - THRUSTER_RANGE * SysReversePowerPercentage / 100;
            Serial.printf("Reverse thruster limit to %i%%\n",arg1);
          }
        break;

        case 502:
          if (arg1 >= 1500 && arg1 <= 2000)
          {
            SysForwardPowerLimit = arg1;
            Serial.printf("Forward thruster limit to %i\n",arg1);
          }
        break;

        case 503:
          if (arg1 >= 1000 && arg1 <= 1500)
          {
            SysReversePowerLimit = arg1;
            Serial.printf("Reverse thruster limit to %i\n",arg1);
          }
        break;

        case 504:
          if (arg1 > 1000)
          {
            SysThrusterUpdateTime = arg1;
            update_thrusters_timer->pause();
            update_thrusters_timer->setOverflow(SysThrusterUpdateTime, MICROSEC_FORMAT);
            update_thrusters_timer->resume();
            Serial.printf("Thruster update time to %i\n", arg1);
          }
        break;

        case 505:
          if (arg1 > 1)
          {
            SysThrusterUpdateInterval = arg1;
            Serial.printf("Thruster update interval (msec) to %i\n", arg1);
          }
        break;

        case 506:
          // stale data timeout
        break;

        case 507:
          SysSigPowerMax = arg1;
          Serial.printf("Recieved signal power upper threshold to %i\n", arg1);
        break;

        case 508:
          SysSigPowerMin = arg1;
          Serial.printf("Recieved signal power lower threshold to %i\n", arg1);
        break;

        case 509:
          if (arg1 >= 0 && arg1 <= DEAD_AHEAD)
          {
            SysAngleWide = arg1;
            Serial.printf("Wide angle to %i\n", arg1);
          }
        break;

        case 510:
          if (arg1 >= 0 && arg1 <= DEAD_AHEAD)
          {
            SysAngleTight = arg1;
            Serial.printf("Tight angle to %i\n", arg1);
          }
        break;

        case 511:
          if (arg1 >= 0 && arg1 <= THRUSTER_RANGE)
          {
            SysForwardOffset = arg1;
            Serial.printf("Thruster forward offset to %i\n", arg1);
          }
        break;

        case 512:
          if (arg1 >= 1 && arg1 <= 10)
          {
            SysOrder = arg1;
            Serial.printf("Forward mode function order to %i\n", arg1);
          }
        break;

        case 513:
          SysDoaOffset = arg1;
          Serial.printf("DOA Offset to %i", SysDoaOffset);
        break;

        case 514:
          if (arg1 == 0 || arg1 == 1)
          {
            SysReflect = arg1;
            Serial.printf("DOA reflect to %i\n", arg1);
          }
        break;

        case 515:
          if (arg1 >= SysSigPowerMin && arg1 <= SysSigPowerMin)
          {
            SysSpeedChangePower = arg1;
            Serial.printf("Speed Change Power to %i\n", arg1);
          }
        break;

        case 516:
          if (arg1 >= 0 && arg1 <= 500)
          {
            SysForwardOffsetFar = arg1;
            Serial.printf("Forward Offset Far to %i\n", arg1);
          }
        break;

        case 517:
          if (arg1 >= 0 && arg1 <= 500)
          {
            SysForwardOffsetNear = arg1;
            Serial.printf("Forward Offset Near to %i\n", arg1);
          }
        break;

        default:
        break;
      }
    break;

    case 3: // arg1 and arg2
      switch(opcode)
      {
        case 600: // manual drive
          PortTarget = THRUSTER_NEUTRAL;
          StarTarget = THRUSTER_NEUTRAL;
          stale_data_timer->pause();
          stale_data_timer->setCount(1);
          move_boat = auto_stop;

          if (arg1 >= THRUSTER_MAX_REVERSE && arg1 <= THRUSTER_MAX_FORWARD &&
              arg2 >= THRUSTER_MAX_REVERSE && arg2 <= THRUSTER_MAX_FORWARD)
          {
            PortTarget = arg1;
            StarTarget = arg2;
          }
          
          Serial.printf("Manual Mode: Port=%i Star=%i\n", PortTarget, StarTarget);
        break;

        case 601:
          if (arg1 >= THRUSTER_NEUTRAL && arg1 <= THRUSTER_MAX_FORWARD &&
              arg2 >= THRUSTER_MAX_REVERSE && arg2 <= THRUSTER_NEUTRAL)
          {
            SysPivotFWValue = arg1;
            SysPivotRVValue = arg2;
            Serial.printf("Set Pivot to FW=%i RV=%i\n", SysPivotFWValue, SysPivotRVValue);
          }
        break;

        case 602:
          if (arg1 >= 0 && arg1 <= 100 &&
              arg2 >= 0 && arg2 <= 100)
          {
            SysPivotFWValue = THRUSTER_NEUTRAL + THRUSTER_RANGE * arg1 / 100;
            SysPivotRVValue = THRUSTER_NEUTRAL - THRUSTER_RANGE * arg2 / 100;
            Serial.printf("Set Pivot to FW=%i(%i%%) RV=%i(%i%%)\n", SysPivotFWValue, arg1, SysPivotRVValue, arg2);
          }
        break;

        default:
        break;
      }

    break;

    default:
    break;
  } 
}


///////////////////////////////////////////////////////////////////////////////
///             Periodic Thruster Updating Functions
///////////////////////////////////////////////////////////////////////////////
void next_thruster_values() {

    int currPortCC = port_timer->getCaptureCompare(port_channel, MICROSEC_COMPARE_FORMAT)+1; // +1 bc setting CCreg is one less than requested.
    int currStarCC = star_timer->getCaptureCompare(star_channel, MICROSEC_COMPARE_FORMAT)+1;
    int change = min(abs((int)PortTarget - currPortCC), SysThrusterUpdateInterval);
    
    if (PortTarget > currPortCC) {
      NextPortValue = min(SysForwardPowerLimit, currPortCC + change);
    }
    else {
      NextPortValue = max(SysReversePowerLimit, currPortCC - change);
    }

    change = min(abs((int)StarTarget - currStarCC), SysThrusterUpdateInterval);
    
    if (StarTarget > currStarCC) {
      NextStarValue = min(SysForwardPowerLimit, currStarCC + change);
    }
    else {
      NextStarValue = max(SysReversePowerLimit, currStarCC - change);
    }

}

void thruster_init() {

  update_thrusters_timer->pause();

  // turn on relay
  digitalWrite(MOTOR_PIN, 1);

  // 2000ms on time
  port_timer->setCaptureCompare(port_channel, THRUSTER_MAX_FORWARD, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, THRUSTER_MAX_FORWARD, MICROSEC_COMPARE_FORMAT);

  port_timer->resume();
  star_timer->resume();

  //delay the time needed for motors to recognize change
  delay(2000);

  // 1500ms on time
  port_timer->setCaptureCompare(port_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);

  delay(1000);

  PortTarget = THRUSTER_NEUTRAL;
  StarTarget = THRUSTER_NEUTRAL;
  NextPortValue = THRUSTER_NEUTRAL;
  NextStarValue = THRUSTER_NEUTRAL;

  update_thrusters_timer->resume();

  Serial.println("Initialized Thrusters");
}

void thruster_stop()
{
  digitalWrite(MOTOR_PIN,0);
  port_timer->pause();
  star_timer->pause();
  port_timer->setCaptureCompare(port_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  update_thrusters_timer->pause();
}

void stale_data()
{
  state = STOP;
  Serial.println("Stale data. Stopping");
}

void update_thrusters() {

  port_timer->setCaptureCompare(port_channel, NextPortValue, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, NextStarValue, MICROSEC_COMPARE_FORMAT);  

  CalculateNext = TRUE;  
}

void checkPiSerial(char recv_data[], uint8_t *msglen, uint8_t *flag) {

  while (PiSerial.available() && *msglen < MAX_MSG_LEN)
  {
    recv_data[*msglen] = PiSerial.read();
    if (recv_data[*msglen] == '\n')
    {
      recv_data[*msglen] = '\0';
      *flag = TRUE;    

    }
    *msglen = *msglen + 1; 
  }
  if (*msglen == MAX_MSG_LEN)
  {
    *flag = TRUE;
  }
}

/*
E-Stop interrupt
will be connect to pin that reads E-stop signal
*/
void E_stop()
{
  //set up flags for interrupt
  static int stop_flag = FALSE;
  static int re_start = FALSE; // changed from TRUE.
  //check if the stop flag is high
  if(stop_flag == FALSE)
  {
    Serial.println("E-Stopped triggered");

    //if low then open relays and turn of motors
    thruster_stop();

    //make stop flag high
    stop_flag = TRUE; 
  }
  //if stop flag high, check re-start flag
  else if(re_start == FALSE)
  {
    //if it is low then make the flag high   
    re_start = TRUE; 
  }
  else if((stop_flag == TRUE) && (re_start == TRUE))
  {
    Serial.println("E-Stop restart triggered");

    //re-initalize motors for use
    thruster_init();       
    //reset flags
    stop_flag = FALSE;
    re_start = FALSE;
    
  }
}

void Check_Switch()
{
  static int oldswitchstate = SWITCH_OFF;
  static int switchstate;
  delay(5);
  switchstate = digitalRead(SWITCH_PIN);
  if (switchstate == SWITCH_ON && oldswitchstate == SWITCH_OFF)
  {
    Serial.println("Turned On");

    digitalWrite(PI_PIN,1); 
    digitalWrite(KRAKEN_PIN,1);

    thruster_init();

    oldswitchstate = switchstate;
    state = GO;
  }
  else if (switchstate == SWITCH_OFF && oldswitchstate == SWITCH_ON)
  {
    Serial.println("Turned Off");
  
    digitalWrite(PI_PIN,0); // TODO: uncomment. commented for testing
    digitalWrite(KRAKEN_PIN,0); // TODO: uncomment. commented for testing
  
    thruster_stop();

    oldswitchstate = switchstate;
    state = STOP;
  }
}




