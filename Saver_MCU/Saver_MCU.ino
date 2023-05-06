#define TRUE            1
#define FALSE           0
#define SIG_POWER_MAX   -140 
#define SIG_POWER_MIN   -600
#define P_K             1 // delete
#define DEAD_AHEAD      180
#define ANGLE_WIDE      30
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

#define DEBUGGING TRUE

typedef enum STATE_TYPE
{
  GO,
  STOP,
}STATE_TYPE;

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
static int SysForwardPowerPercentage = 20;
static int SysReversePowerPercentage = 20;
static int SysForwardPowerLimit = THRUSTER_NEUTRAL + THRUSTER_RANGE * SysForwardPowerPercentage / 100;
static int SysReversePowerLimit = THRUSTER_NEUTRAL - THRUSTER_RANGE * SysReversePowerPercentage / 100;
static int SysPivotFWValue = 1600;
static int SysPivotRVValue = 1400;
static int SysManualPort = THRUSTER_NEUTRAL;
static int SysManualStar = THRUSTER_NEUTRAL;
static int SysThrusterUpdateTime = THRUSTER_UPDATE_TIME;
static int SysThrusterUpdateInterval = THRUSTER_UPDATE_MAX_INTERVAL;
static int SysStaleDataTimeout = STALE_DATA_TIMEOUT;
static int SysSigPowerMax = SIG_POWER_MAX;
static int SysSigPowerMin = SIG_POWER_MIN;
static int SysAngleWide = ANGLE_WIDE;
static int SysAngleTight = ANGLE_TIGHT;
static int SysForwardOffset = 50;
static int SysOrder = 2;
static int SysReflect = 1;
static int SysDoaOffset = 225;

// Swappable function pointers
static int32_t (*process_data)(int32_t, int32_t, int32_t, uint8_t); // preprocessing on data
static void (*move_boat)(int32_t); // movement control algorithm

///////////////////////////////////////////////////////////////////////////////
///                          Movement Modes
///////////////////////////////////////////////////////////////////////////////

void move_stop(int32_t doa)
{
  PortTarget = THRUSTER_NEUTRAL;
  StarTarget = THRUSTER_NEUTRAL;
}

void pivot_cw(int32_t doa)
{
  if (doa > DEAD_AHEAD - SysAngleTight && doa < DEAD_AHEAD + SysAngleTight)
  {
    PortTarget = THRUSTER_NEUTRAL;
    StarTarget = THRUSTER_NEUTRAL;
    move_boat = auto_forward;
  }
  else
  {
    PortTarget = SysPivotFWValue;
    StarTarget = SysPivotRVValue;
  }
  dbprint("Pivot CW => ");
}

void pivot_ccw(int32_t doa)
{
  if (doa > DEAD_AHEAD - SysAngleTight && doa < DEAD_AHEAD + SysAngleTight)
  {
    PortTarget = THRUSTER_NEUTRAL;
    StarTarget = THRUSTER_NEUTRAL;
    move_boat = auto_forward;
  }
  else
  {
    PortTarget = SysPivotRVValue;
    StarTarget = SysPivotFWValue;
  }
  dbprint("Pivot CCW => ");
}

void auto_forward(int32_t doa)
{

  int32_t offset;

  if (doa < DEAD_AHEAD - SysAngleWide)
  {
    PortTarget = THRUSTER_NEUTRAL;
    StarTarget = THRUSTER_NEUTRAL;
    move_boat = pivot_cw;
    return;
  }
  if (doa > DEAD_AHEAD + SysAngleWide)
  {
    PortTarget = THRUSTER_NEUTRAL;
    StarTarget = THRUSTER_NEUTRAL;
    move_boat = pivot_cw;
  }

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
  dbprint("Move forward => ");
}

void move_manual (int32_t)
{
  PortTarget = SysManualPort;
  StarTarget = SysManualStar;
  dbprint("manual move =>");
}

///////////////////////////////////////////////////////////////////////////////
///                      Preprocessing Functions
///////////////////////////////////////////////////////////////////////////////
int32_t single_doa(int32_t doa, int32_t conf, int32_t power, uint8_t reset)
{
  int result = abs(360*SysReflect - ((doa + SysDoaOffset) % 360));
  Serial.printf("doa=%d offset=%d res=%d", doa, SysDoaOffset, result);
  return result;
}

int32_t window_doa(int32_t doa, int32_t conf, int32_t power, uint8_t reset)
{
  static const uint8_t window_size = 8;
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

  Serial.printf("doa=%d,pow=%d  offset=%d %d\n", doa, power, SysDoaOffset, SysReflect);

  return result;
}

static int32_t (*preprocessors[2])(int32_t, int32_t, int32_t, uint8_t) = 
{
  single_doa,
  window_doa
};

void (*move_modes[5])(int32_t) =
{
  pivot_cw,
  pivot_ccw,
  auto_forward,
  move_manual  
};

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
  move_boat = auto_forward;

  dbprintln("Setup Complete2");

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
        state = STOP;
        break;
      }
      else if(power < SysSigPowerMin) // beacon off
      {
        break;
      }
      else // new data to process
      {
        stale_data_timer->pause();
        stale_data_timer->setCount(1);
        stale_data_timer->resume();

        processed_doa = process_data(doa, conf, power, FALSE);
        if (processed_doa >= 0 && processed_doa < 360)
        {
          move_boat(processed_doa);
          Serial.printf("P: %d   S: %d\n", PortTarget, StarTarget);
        }
      }

    break;

    case STOP:
      StarTarget = THRUSTER_NEUTRAL;
      PortTarget = THRUSTER_NEUTRAL;
      if (NewPiData && power < SysSigPowerMax)
      {
        state = GO;
      }
    break;

    default:
    break; 
  } 
}

void config_system(int32_t opcode, int32_t arg1, int32_t arg2, int status)
{
  switch (status)
  {
    case 1: // no arg1 or arg2
      switch(opcode)
      {
        case 400:
          move_boat = move_stop;
          dbprintln("Move Mode: Stop");
        break;
        case 401:
          move_boat = auto_forward;
          dbprintln("Move Mode: Auto");
        break;
        case 402:
          move_boat = move_manual;
          dbprintln("Move Mode: Manual");
        break;
        case 410:
          process_data = single_doa;
          dbprintln("Process Mode: Single");
        break;
        case 411:
          process_data = window_doa;
          dbprintln("Process Mode: Window");
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
            printf("Change F Pow"); 
          }
        break;
        case 501:
          if (arg1 >= 0 && arg1 <= 100)
          {
            SysReversePowerPercentage = arg1;
            SysReversePowerLimit = THRUSTER_NEUTRAL - THRUSTER_RANGE * SysReversePowerPercentage / 100;
          }
        break;
        case 502:
          if (arg1 >= 1500 && arg1 <= 2000)
            SysForwardPowerLimit = arg1;
        break;
        case 503:
          if (arg1 >= 1000 && arg1 <= 1500)
            SysReversePowerLimit = arg1;
        break;
        case 504:
          if (arg1 > 1000)
          {
            SysThrusterUpdateTime = arg1;
            update_thrusters_timer->pause();
            update_thrusters_timer->setOverflow(SysThrusterUpdateTime, MICROSEC_FORMAT);
            update_thrusters_timer->resume();
          }
        break;
        case 505:
          if (arg1 > 1)
            SysThrusterUpdateInterval = arg1;
        break;
        case 506:
          // stale data timeout
        break;
        case 507:
          SysSigPowerMax = arg1;
        break;
        case 508:
          SysSigPowerMin = arg1;
        break;
        case 509:
          if (arg1 >= 0 && arg1 <= 180)
          {
            SysAngleWide = arg1;
          }
        break;
        case 510:
          if (arg1 >= 0 && arg1 <= 180)
          {
            SysAngleTight = arg1;
          }
        break;
        case 511:
          if (arg1 >= 0 && arg1 <= 500)
          {
            SysForwardOffset = arg1;
          }
          break;
        case 512:
          if (arg1 >= 1 && arg1 <= 10)
          {
            SysOrder = arg1;
          }
          break;
        case 513:
          SysDoaOffset = arg1;
          Serial.printf("DOA Offset = %d", SysDoaOffset);
        break;
        case 514:
          if (arg1 == 0 || arg1 == 1)
          {
            SysReflect = arg1;
          }
        case 515:
          if (arg1 >= 0 && arg1 <= 3)
            move_boat = move_modes[arg1];
        default:
        break;
      }
    break;

    case 3: // arg1 and arg2
      switch(opcode)
      {
        case 600:
          SysManualPort = arg1;
          SysManualStar = arg2;
          printf("Manual Port=%i Star=%i\n", SysManualPort, SysManualStar);
        break;
        case 601:
          SysPivotFWValue = arg1;
          SysPivotRVValue = arg2;
          printf("Pivot Port=%i Star=%i\n", SysPivotFWValue, SysPivotRVValue);
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

  dbprintln("Initialized Thrusters");
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
    //if low then open relays and turn of motors
    thruster_stop();

    //make stop flag high
    stop_flag = TRUE;
    dbprintln("E-Stopped triggered");
  }
  //if stop flag high, check re-start flag
  else if(re_start == FALSE)
  {
    //if it is low then make the flag high   
    re_start = TRUE; 
  }
  else if((stop_flag == TRUE) && (re_start == TRUE))
  {
    //re-initalize motors for use
    thruster_init();       
    //reset flags
    stop_flag = FALSE;
    re_start = FALSE;
    dbprintln("E-Stop restart triggered");
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
    dbprintln("Turned On");

    digitalWrite(PI_PIN,1); 
    digitalWrite(KRAKEN_PIN,1);

    thruster_init();

    oldswitchstate = switchstate;
    state = GO;
  }
  else if (switchstate == SWITCH_OFF && oldswitchstate == SWITCH_ON)
  {
    dbprintln("Turned Off");
  
    digitalWrite(PI_PIN,0); // TODO: uncomment. commented for testing
    digitalWrite(KRAKEN_PIN,0); // TODO: uncomment. commented for testing
  
    thruster_stop();

    oldswitchstate = switchstate;
    state = STOP;
  }
}

void dbprint(char *str)
{
  if (DEBUGGING)
  {
    Serial.print(str);
  }
}

void dbprintln(char *str)
{
  if (DEBUGGING)
  {
    Serial.println(str);
  }
}



