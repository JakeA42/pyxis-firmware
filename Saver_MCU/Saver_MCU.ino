#define TRUE            1
#define FALSE           0
#define NUM_BYTES       20
#define RIGHT_MOT       4
#define LEFT_MOT        3
#define PWM_FREQ        400
#define PWM_MAX_FW      80
#define PWM_MAX_RV      40 
#define PWM_STOP        60 
#define PWR_THRESH_MAX  -140 
#define PWR_THRESH_MIN  -600
#define P_K             1
#define LEFT_MIN        185
#define LEFT_MAX        355
#define RIGHT_MAX       175
#define RIGHT_MIN       5
#define ERROR_MAX       170
#define ERROR_MIN       0
#define DEAD_AHEAD      180

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

uint32_t PortTarget = THRUSTER_NEUTRAL;
uint32_t StarTarget = THRUSTER_NEUTRAL;
uint32_t NextPortValue = THRUSTER_NEUTRAL;
uint32_t NextStarValue = THRUSTER_NEUTRAL;
uint8_t CalculateNext = FALSE;

//attach serial to D0 (Rx) and D1 (tx)
HardwareSerial PiSerial(D0, D1);

//starting state for the state machine
//didn't want to make it global but arduino is stupid
STATE_TYPE state = STOP;

int standby_flag = 0;
int start_flag = 0;



void setup() {
  Serial.begin(115200); // PC
  PiSerial.begin(115200); // Pi

  // Relays
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(PI_PIN, OUTPUT);
  pinMode(KRAKEN_PIN, OUTPUT);

  delay(5000);

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
  
  dbprintln("Setup Complete");

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
  static uint32_t doa = 0;
  static uint32_t conf = 0;  
  static uint32_t processed_doa = DEAD_AHEAD;
  static uint8_t filled_window;

  static int ForwardPowerPercentage = 100;
  static int ReversePowerPercentage = 100;
  static int ForwardPowerLimit = THRUSTER_NEUTRAL + THRUSTER_RANGE * ForwardPowerPercentage / 100;
  static int ReversePowerLimit = THRUSTER_NEUTRAL - THRUSTER_RANGE * ReversePowerPercentage / 100;

  static uint32_t (*process_data)(uint32_t, uint32_t, int32_t, uint8_t) = single_doa; // preprocessing on data
  static void (*move_boat)(uint32_t, int, int) = pivot_proportional; // movement control algorithm

  int status = 0;

  NewPiData = FALSE;
  checkPiSerial(PiMsg, &PiMsgLen, &PiMsgReady);
  if (PiMsgReady)
  {
    dbprintln(PiMsg);
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
    next_thruster_values(ForwardPowerLimit, ReversePowerLimit);
    CalculateNext = FALSE;    
  }

  switch(state)
  {
    case GO:

      if (!NewPiData)
      {
        break;
      }

      if(power > PWR_THRESH_MAX) // close to target
      {
        state = STOP;
        break;
      }
      else if(power < PWR_THRESH_MIN) // beacon off
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
          move_boat(processed_doa, ForwardPowerLimit, ReversePowerLimit);
          Serial.printf("P: %d   S: %d\n", PortTarget, StarTarget);
        }
      }

    break;

    case STOP:
      StarTarget = THRUSTER_NEUTRAL;
      PortTarget = THRUSTER_NEUTRAL;
    break;

    default:
    break; 
  } 
}

void config_system(uint32_t opcode, uint32_t arg1, int32_t arg2, int status)
{
  switch (status)
  {
    case 1: // no arg1 or arg2


    break;
    case 2: // arg1
      switch(opcode)
      {
        case 400:
          
        break;

      }

    break;

    case 3: // arg1 and arg2
      

    break;

    default:
    break;
  }
  
}

///////////////////////////////////////////////////////////////////////////////
///                          Movement Modes
///////////////////////////////////////////////////////////////////////////////
void pivot_cw(uint32_t doa, int forwardlimit, int reverselimit)
{
  PortTarget = forwardlimit;
  StarTarget = reverselimit;
  dbprint("Pivot CW => ");
}

void pivot_ccw(uint32_t doa, int forwardlimit, int reverselimit)
{
  PortTarget = reverselimit;
  StarTarget = forwardlimit;
  dbprint("Pivot CCW => ");
}

void pivot_proportional(uint32_t doa, int forwardlimit, int reverselimit)
{
  // may want to use some other variables than forwardlimit and reverse limit

  if (doa >= DEAD_AHEAD)
  {
    // may have these backwards
    PortTarget = THRUSTER_NEUTRAL + (doa - DEAD_AHEAD) * (forwardlimit - THRUSTER_NEUTRAL) / DEAD_AHEAD;
    StarTarget = THRUSTER_NEUTRAL - (doa - DEAD_AHEAD) * (THRUSTER_NEUTRAL - reverselimit) / DEAD_AHEAD;
  }
  else
  {
    PortTarget = THRUSTER_NEUTRAL - (doa - DEAD_AHEAD) * (THRUSTER_NEUTRAL - reverselimit) / DEAD_AHEAD;
    StarTarget = THRUSTER_NEUTRAL + (doa - DEAD_AHEAD) * (forwardlimit - THRUSTER_NEUTRAL) / DEAD_AHEAD;
  }
  dbprint("Pivot Proportional => ");
}

void pivot_quadratic(uint32_t doa, int forwardlimit, int reverselimit)
{
  // may want to use some other variables than forwardlimit and reverse limit

  if (doa >= DEAD_AHEAD)
  {
    // may have these backwards
    PortTarget = THRUSTER_NEUTRAL + 
                (doa - DEAD_AHEAD) * (forwardlimit - THRUSTER_NEUTRAL) / DEAD_AHEAD / THRUSTER_RANGE + 
                (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (forwardlimit - THRUSTER_NEUTRAL) / DEAD_AHEAD / DEAD_AHEAD;
    StarTarget = THRUSTER_NEUTRAL - 
                (doa - DEAD_AHEAD) * (THRUSTER_NEUTRAL - reverselimit) / DEAD_AHEAD / THRUSTER_RANGE - 
                (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (THRUSTER_NEUTRAL - reverselimit) / DEAD_AHEAD / DEAD_AHEAD;
  }
  else
  {
    PortTarget = THRUSTER_NEUTRAL - 
                (doa - DEAD_AHEAD) * (THRUSTER_NEUTRAL - reverselimit) / DEAD_AHEAD / THRUSTER_RANGE - 
                (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (THRUSTER_NEUTRAL - reverselimit) / DEAD_AHEAD / DEAD_AHEAD;
    StarTarget = THRUSTER_NEUTRAL + 
                (doa - DEAD_AHEAD) * (forwardlimit - THRUSTER_NEUTRAL) / DEAD_AHEAD / THRUSTER_RANGE + 
                (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (doa - DEAD_AHEAD) * (forwardlimit - THRUSTER_NEUTRAL) / DEAD_AHEAD / DEAD_AHEAD;
  }
  dbprint("Pivot Proportional => ");
}

void pivot_exponential(uint32_t doa, int forwardlimit, int reverselimit)
{
  uint8_t exp = 3; // make parameter
  uint32_t offset;

  offset = THRUSTER_RANGE * pow(((double)doa - DEAD_AHEAD) / DEAD_AHEAD, exp);

  if (doa >= DEAD_AHEAD)
  {
    // may have these backwards
    PortTarget = THRUSTER_NEUTRAL + offset;
    StarTarget = THRUSTER_NEUTRAL - offset;
  }
  else
  {
    PortTarget = THRUSTER_NEUTRAL - offset;
    StarTarget = THRUSTER_NEUTRAL + offset;
  }
  dbprint("Pivot Exponential => ");
}

static void (*move_boat[])(uint32_t, int, int) 
{
  pivot_cw,
  pivot_ccw,
  pivot_proportional,
  pivot_quadratic,
  pivot_exponential
};

// void pwm_calc(uint32_t doa)
// {
//   //variables for calculating proportional speed
//   int error = 0;
//   int calc_speed = 0;
//   uint32_t pwm_value = 0;
  
//   //remap doa angle so 45 = 0
//   int angle = doa - 45;
//   //check if the angle is negative, offset if it is

//   if(angle < 0)
//   {
//     angle += 360;
//   }

//   //Check if boat needs to turn right or left
//   if((RIGHT_MIN <= angle) && (angle <= RIGHT_MAX))  //turn right
//   {
//     //calculate the error
//     error = angle;

//     //get the proportional speed
//     calc_speed = proportional_calc(error);

//     //damping the power on an exponential curve
//     //TODO: either take out or change calculation
//     //calc_speed = sqrt(calc_speed);  

//     //remap power level to pwm duty cycle range (60 - 80)
//     pwm_value = map(calc_speed, 0, 100, PWM_STOP, PWM_MAX_FW);
  
//     //change PWM values
//     update_thrusters(pwm_value,PWM_MAX_FW); //TODO: check if right motor should be dampened

//   }  
//   else if((LEFT_MIN <= angle) && (angle <= LEFT_MAX)) //turn left
//   {
//     //compute error
//     error = LEFT_MAX - angle;
    
//     //get the calculated speed vvalue
//     calc_speed = proportional_calc(error);

//     //damping the power on an exponential curve
//     //TODO: either take out or change calculation
//     //calc_speed = sqrt(calc_speed); 

//     //remap power level to pwm duty cycle range (60 - 80)
//     pwm_value = map(calc_speed, 0, 100, PWM_STOP, PWM_MAX_FW);
  
//     //change duty cycles of the left and right motor
//     update_thrusters(PWM_MAX_FW, pwm_value);  //TODO: check if left value needs to be dampened

//   }
//   else if((angle < RIGHT_MIN) && (angle > LEFT_MIN))//edge case where boat is backwards
//   {
//     //change right motor to full power and left motor off
//     update_thrusters(PWM_STOP, PWM_MAX_FW);     
//   }
//   else //boat is headed within the right direction keep moving forward
//   {
//     update_thrusters(PWM_MAX_FW,PWM_MAX_FW);  
//   }
// }


///////////////////////////////////////////////////////////////////////////////
///                      Preprocessing Functions
///////////////////////////////////////////////////////////////////////////////
uint32_t single_doa(uint32_t doa, uint32_t conf, int32_t power, uint8_t reset)
{
  return doa;
}

uint32_t window_doa(uint32_t doa, uint32_t conf, int32_t power, uint8_t reset)
{
  static const uint8_t window_size = 8;
  static uint32_t window[window_size] = {DEAD_AHEAD};
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

  uint32_t sum = 0;
  for(int i = 0; i < window_size; i++)
  {
    sum += window[i];
  }

  return sum / window_size;
}

uint32_t (*preprocessors[])(uint32_t, uint32_t, int32_t, uint8_t) = 
{
  single_doa,
  window_doa
};

///////////////////////////////////////////////////////////////////////////////
///             Periodic Thruster Updating Functions
///////////////////////////////////////////////////////////////////////////////
void next_thruster_values(int ForwardPowerLimit, int ReversePowerLimit) {

    int currPortCC = port_timer->getCaptureCompare(port_channel, MICROSEC_COMPARE_FORMAT)+1; // +1 bc setting CCreg is one less than requested.
    int currStarCC = star_timer->getCaptureCompare(star_channel, MICROSEC_COMPARE_FORMAT)+1;
    int change = min(abs((int)PortTarget - currPortCC), THRUSTER_UPDATE_MAX_INTERVAL);
    
    if (PortTarget > currPortCC) {
      NextPortValue = min(ForwardPowerLimit, currPortCC + change);
    }
    else {
      NextPortValue = max(ReversePowerLimit, currPortCC - change);
    }

    change = min(abs((int)StarTarget - currStarCC), THRUSTER_UPDATE_MAX_INTERVAL);
    
    if (StarTarget > currStarCC) {
      NextStarValue = min(ForwardPowerLimit, currStarCC + change);
    }
    else {
      NextStarValue = max(ReversePowerLimit, currStarCC - change);
    }

}

void thruster_init() {

  dbprintln("Initializing Thrusters");

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
* Takes in the doa error and then multiples it by the proportional constant.
* once done it will map the speed to 0-100 range.
*/
uint32_t proportional_calc(int error)
{
  uint32_t calc_speed;
  calc_speed = error * P_K;    

  if(calc_speed > 170)
  {
    calc_speed = 170;
  }
  //remap to power damping range (0 - 100)
  calc_speed = map(calc_speed, ERROR_MIN, ERROR_MAX, 0, 100);

  return calc_speed;
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
  
    //digitalWrite(PI_PIN,0); // TODO: uncomment. commented for testing
    //digitalWrite(KRAKEN_PIN,0); // TODO: uncomment. commented for testing
  
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



