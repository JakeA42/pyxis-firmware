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
#define WINDOW_SIZE     8

#define PORT_PIN      D10
#define STAR_PIN      D9
#define PI_PIN        A5
#define KRAKEN_PIN    A4
#define E_STOP_PIN    D6
#define MOTOR_PIN     PB15  //COPI pin
#define OFF_PIN       D11
#define START_PIN     D12

#define MSG_BUFF_SIZE 24
#define MAX_MSG_LEN (MSG_BUFF_SIZE - 1)

#define THRUSTER_MAX_REVERSE 1000 // microsec
#define THRUSTER_NEUTRAL 1500 // microsec
#define THRUSTER_MAX_FORWARD 2000 // microsec
#define THRUSTER_PWM_PERIOD 4000 //microsec
#define THRUSTER_RANGE (THRUSTER_MAX_FORWARD - THRUSTER_NEUTRAL)
#define THRUSTER_UPDATE_MAX_INTERVAL 2 // microsec. 1 breaks it
#define THRUSTER_UPDATE_TIME 7500 // microsec

typedef enum STATE_TYPE
{
  WAIT_FOR_EVENT,
  VALIDATE_DATA,
  SPD_ADJ,
  STOP,
  NO_STATE
}STATE_TYPE;

//stuff needed for timers to do PWM
TIM_TypeDef *tim1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PORT_PIN), PinMap_PWM);  //TIM4 channel 3
TIM_TypeDef *tim2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(STAR_PIN), PinMap_PWM);  //TIM4 channel 4 (or at least it should be)
HardwareTimer *port_timer = new HardwareTimer(tim1);  //TIM4 channel 3
HardwareTimer *star_timer = new HardwareTimer(tim2);  //TIM4 channel 4
uint32_t port_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PORT_PIN), PinMap_PWM));
uint32_t star_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(STAR_PIN), PinMap_PWM));

uint32_t PortTarget = THRUSTER_NEUTRAL;
uint32_t StarTarget = THRUSTER_NEUTRAL;
uint32_t NextPortValue = THRUSTER_NEUTRAL;
uint32_t NextStarValue = THRUSTER_NEUTRAL;
uint8_t CalculateNext = FALSE;

//attach serial to D0 (Rx) and D1 (tx)
HardwareSerial PiSerial(D0, D1);

//starting state for the state machine
//didn't want to make it global but arduino is stupid
STATE_TYPE state = NO_STATE;

int standby_flag = 0;
int start_flag = 0;

void setup() {
  delay(5000);

  Serial.begin(115200); // PC
  PiSerial.begin(115200); // Pi


  //set up motor relay pin
  pinMode(MOTOR_PIN, OUTPUT); 
  
  //set up pi relay pin
  pinMode(PI_PIN, OUTPUT); 
    
  //set up kraken pin
  pinMode(KRAKEN_PIN, OUTPUT);    

  pinMode(OFF_PIN, INPUT_PULLUP);

  pinMode(START_PIN, INPUT_PULLUP);
  
  //E-stop interrupt initalization 
  pinMode(E_STOP_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN),E_stop,FALLING);
  attachInterrupt(digitalPinToInterrupt(OFF_PIN),Turn_Off,FALLING);
  attachInterrupt(digitalPinToInterrupt(START_PIN),Turn_On,FALLING);

  // Setup thruster PWMs  
  port_timer->pause();
  star_timer->pause();  
  port_timer->setMode(port_channel, TIMER_OUTPUT_COMPARE_PWM1, PORT_PIN);
  star_timer->setMode(star_channel, TIMER_OUTPUT_COMPARE_PWM1, STAR_PIN);
  port_timer->setOverflow(THRUSTER_PWM_PERIOD, MICROSEC_FORMAT);
  star_timer->setOverflow(THRUSTER_PWM_PERIOD, MICROSEC_FORMAT);
  port_timer->setCaptureCompare(port_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  port_timer->resume();
  star_timer->resume();

  TIM_TypeDef *tim3 = TIM3;
  HardwareTimer *update_thrusters_timer = new HardwareTimer(tim3);
  update_thrusters_timer->setOverflow(THRUSTER_UPDATE_TIME, MICROSEC_FORMAT);
  update_thrusters_timer->attachInterrupt(update_thrusters);
  update_thrusters_timer->resume();

}

void loop() {
  
  //TODO:check to see if this will cause issues. Might change them to static
  static char PiMsg[MSG_BUFF_SIZE] = {0};
  static char PcMsg[MSG_BUFF_SIZE] = {0};
  static uint8_t PiMsgLen = 0;
  static uint8_t PcMsgLen = 0;
  static uint8_t PiMsgReady = FALSE;
  static uint8_t PcMsgReady = FALSE;

  static uint32_t doa_window[WINDOW_SIZE] = {180};
  static int32_t power = 0; 
  static uint32_t doa = 0;
  static uint32_t conf = 0;  
  static uint32_t num_samples = 0;
  static uint32_t windowed_doa = 180;
  static uint8_t filled_window;

  static int ForwardPowerPercentage = 100;
  static int ReversePowerPercentage = 100;
  static int ForwardPowerLimit = THRUSTER_NEUTRAL + THRUSTER_RANGE * ForwardPowerPercentage / 100;
  static int ReversePowerLimit = THRUSTER_NEUTRAL - THRUSTER_RANGE * ReversePowerPercentage / 100;
  static int change = 0;

  checkPiSerial(PiMsg, &PiMsgLen, &PiMsgReady);
  if (PiMsgReady)
  {
    Serial.println(PiMsg);
    sscanf(PiMsg, "%i,%i,%i", &doa, &conf, &power);
    state = VALIDATE_DATA;
  }

  if (CalculateNext) {
    int currPortCC = port_timer->getCaptureCompare(port_channel, MICROSEC_COMPARE_FORMAT)+1; // +1 bc setting CCreg is one less than requested.
    int currStarCC = star_timer->getCaptureCompare(star_channel, MICROSEC_COMPARE_FORMAT)+1;

    change = min(abs((int)PortTarget - currPortCC), THRUSTER_UPDATE_MAX_INTERVAL);
    
    if (PortTarget > currPortCC) {
      NextPortValue = min(ForwardPowerLimit, currPortCC + change);
    }
    else {
      NextPortValue = max(ReversePowerLimit, currPortCC - change);
    }
    
    // Debug: change value predictably inconsistent. 
    //        Either changes by THRUSTER_UPDATE_MAX_INTERVAL (correct)
    //        or THRUSTER_UPDATE_MAX_INTERVAL-1 (incorrect).
    if (change) {
      Serial.println();
      Serial.println(currPortCC);      
      Serial.println(change);
      Serial.println(NextPortValue);
    }

    change = min(abs((int)StarTarget - currStarCC), THRUSTER_UPDATE_MAX_INTERVAL);
    
    if (StarTarget > currStarCC) {
      NextStarValue = min(ForwardPowerLimit, currStarCC + change);
    }
    else {
      NextStarValue = max(ReversePowerLimit, currStarCC - change);
    }

    CalculateNext = FALSE;    
  }

  //Serial.println("Standvy Pin:");
  //Serial.println(digitalRead(OFF_PIN));
  switch(state)
  {
    case WAIT_FOR_EVENT:
      break;
    case VALIDATE_DATA:
      
      /*Serial.print("UART_DATA: ");
      Serial.print(data);
      Serial.print("\n");*/
      
      //check power data to determine which state to go to
      // if(power > PWR_THRESH_MAX)
      // {
      //   state = STOP;       
      // }
      // //power is too low so ignore that signal
      // else if(power < PWR_THRESH_MIN)
      // {
      //   state = WAIT_FOR_EVENT;
      // }
      // else
      // {
        state = SPD_ADJ;
        ++num_samples;
      // }
    break;      

    case SPD_ADJ: 
      //calculate and set motor speeds
      doa_window[num_samples % 8] = doa;
      windowed_doa = average_window(doa_window, WINDOW_SIZE);
      
      /*Serial.print("index: ");
      Serial.print(num_samples % 8);
      Serial.print("\n"); 
      Serial.print("averaged value: ");
      Serial.print(windowed_doa);
      Serial.print("\n");*/
      
      if(num_samples >= 8)
      {
        new_pwm_calc(windowed_doa);
        filled_window = 1;        
      }
      //go to get_data state
      CalculateNext = TRUE;
      state = WAIT_FOR_EVENT;
    break;

    case STOP:
      //TODO: fix
      //change timer count to neutral timing
      StarTarget = THRUSTER_NEUTRAL;
      PortTarget = THRUSTER_NEUTRAL;

      //go back to GET_DATA
      state = WAIT_FOR_EVENT;
      
    break;

    default:
    break; 
  } 
}

/*
motor ESC init sequence 
*/
void motor_esc_init() {

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN,1);

  delay(1000);

  //create 400Hz pwm and give max speed signal
  port_timer->setCaptureCompare(port_channel, THRUSTER_MAX_FORWARD, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, THRUSTER_MAX_FORWARD, MICROSEC_COMPARE_FORMAT);

  //delay the time needed for motors to recognize change
  delay(1000);

  //give mid frequency stop signal
  port_timer->setCaptureCompare(port_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, THRUSTER_NEUTRAL, MICROSEC_COMPARE_FORMAT);
}

/*
PWM signal calculation function
calculates the counter value for the capture compare channels then sets it
*/

void new_pwm_calc(uint32_t doa)
{
  PortTarget = THRUSTER_NEUTRAL + doa;
  StarTarget = THRUSTER_NEUTRAL - doa;
}

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
    *msglen++; 
  }
  if (*msglen == MAX_MSG_LEN)
  {
    *flag = TRUE;
  }
}

// void checkPcSerial(char recv_data[], uint8_t *msglen, uint8_t *flag) {

//   while (PcSerial.available() && msglen < MAX_MSG_LEN)
//   {
//     recv_data[*msglen] = Serial.read();

//     if (recv_data[*msglen] == '\n')
//     {
//       recv_data[*msglen] = '\0';
//       *flag = TRUE;    

//     }
//     *msglen++; 
//   }
//   if (*msglen == MAX_MSG_LEN)
//   {
//     *flag = TRUE;
//   }
// }


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
  static int re_start = TRUE;
  //check if the stop flag is high
  if(stop_flag == FALSE)
  {
    //if low then open relays and turn of motors
    digitalWrite(MOTOR_PIN,0); //TODO: get the right pin number

    //make stop flag high
    stop_flag = TRUE;
    Serial.println("E-Stopped triggered");
  }
  //if stop flag high, check re-start flag
  else if(re_start == FALSE)
  {
    //if it is low then make the flag high   
    re_start = TRUE; 
  }
  else if((stop_flag == TRUE) && (re_start == TRUE))
  {
    //close the relays to turn the motors back on
    digitalWrite(MOTOR_PIN,1); //TODO: get the right pin number 
    //re-initalize motors for use
    motor_esc_init();       
    //reset flags
    stop_flag = FALSE;
    re_start = FALSE;
    Serial.println("E-Stop restart triggered");
  }
}

void Turn_On()
{
  Serial.println("Turned On");

  digitalWrite(PI_PIN,1); 

  //turn on kraken
  digitalWrite(KRAKEN_PIN,1);

  //turn on motors
  digitalWrite(MOTOR_PIN,1);

  motor_esc_init();

  state = WAIT_FOR_EVENT;
}

void Turn_Off()
{
  Serial.println("Turned Off");
  
  digitalWrite(PI_PIN,0); 

  //turn on kraken
  digitalWrite(KRAKEN_PIN,0);

  //turn on motors
  digitalWrite(MOTOR_PIN,0);

  state = NO_STATE;
}

uint32_t average_window(uint32_t * window, uint32_t window_size)
{
  uint32_t sum = 0;
  for(int i = 0; i < window_size; i++)
  {
    sum += window[i];
  }
  return sum / window_size;
}



