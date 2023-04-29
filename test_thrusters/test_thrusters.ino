// Use Serial Monitor to change thruster speed.

#define TRUE 1
#define FALSE 0
#define NUM_BYTES 20
#define RIGHT_MOT 4
#define LEFT_MOT 3
#define PWM_FREQ 400
#define PWM_MAX_FW 80
#define PWM_MAX_RV 40
#define PWM_STOP 60
#define PWR_THRESH -140
#define P_K 1
#define LEFT_MIN 185
#define LEFT_MAX 355
#define RIGHT_MAX 175
#define RIGHT_MIN 5
#define ERROR_MAX 170
#define ERROR_MIN 0

#define MSG_BUFF_SIZE 24
#define MAX_MSG_LEN (MSG_BUFF_SIZE - 1)

bool NewMessage = FALSE;

#define THRUSTER_MAX_REVERSE 1000 // microsec
#define THRUSTER_NEUTRAL 1500 // microsec
#define THRUSTER_MAX_FORWARD 2000 // microsec
#define THRUSTER_PWM_PERIOD 4000 //microsec
#define THRUSTER_RANGE (THRUSTER_MAX_FORWARD - THRUSTER_NEUTRAL)

#define MOTOR_PIN PB15  //COPI pin
#define PORT_PIN D10
#define STAR_PIN D9

#define THRUSTER_UPDATE_MAX_INTERVAL 2 // microsec. 1 breaks it
#define THRUSTER_UPDATE_TIME 7500 // microsec

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
bool CalculateNext = FALSE;

void setup() {

  //set up baud rate for UART
  Serial.begin(115200);

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

  //motor ESC initializtion sequence
  motor_esc_init();

  TIM_TypeDef *tim3 = TIM3;
  HardwareTimer *update_thrusters_timer = new HardwareTimer(tim3);
  update_thrusters_timer->setOverflow(THRUSTER_UPDATE_TIME, MICROSEC_FORMAT);
  update_thrusters_timer->attachInterrupt(update_thrusters);
  update_thrusters_timer->resume();

  Serial.println("Usage: <port_pwm_msec>,[star_pwm_msec]");
  Serial.println();
  Serial.println("Input range:  1000-2000");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("Stop:         1500,1500");
  Serial.println("Full Forward: 2000,2000");
  Serial.println("Full Reverse: 1000,1000");
  Serial.println();
  Serial.println("Note: Tolerates out of range positive values.");
  Serial.println("      Does not tolerate negative values.");
}

void loop() {

  static bool isfullmsg = FALSE;
  //TODO:check to see if this will cause issues. Might change them to static
  static char msg[MSG_BUFF_SIZE] = {0};
  static uint8_t msglen = 0;

  static int ForwardPowerPercentage = 100;
  static int ReversePowerPercentage = 100;
  static int ForwardPowerLimit = THRUSTER_NEUTRAL + THRUSTER_RANGE * ForwardPowerPercentage / 100;
  static int ReversePowerLimit = THRUSTER_NEUTRAL - THRUSTER_RANGE * ReversePowerPercentage / 100;
  static int change = 0;  
  
  //call the UART function to get all of the info needed
  msglen = UART(msg, msglen);
  if (NewMessage)
  {
    Serial.println(msg);
    sscanf(msg, "%i,%i", &PortTarget, &StarTarget);
    NewMessage = FALSE;
    msglen = 0;
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
  
}

void update_thrusters() {
  
  port_timer->setCaptureCompare(port_channel, NextPortValue, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, NextStarValue, MICROSEC_COMPARE_FORMAT);  

  CalculateNext = TRUE;  
}

int UART(char recv_data[], uint8_t msglen) {

  while (Serial.available() && msglen < MAX_MSG_LEN)
  {
    recv_data[msglen] = Serial.read();

    if (recv_data[msglen] == '\n')
    {
      recv_data[msglen] = '\0';
      NewMessage = TRUE;    
      return msglen;
    }
    msglen++; 
  }
  if (msglen == MAX_MSG_LEN)
  {
    NewMessage = TRUE;
  }
  return msglen;
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
// void pwm_calc(uint32_t doa) {
//   //variables for calculating proportional speed
//   int error = 0;
//   int calc_speed = 0;
//   uint32_t pwm_value = 0;

//   //remap doa angle so 45 = 0
//   int angle = doa - 45;
//   //check if the angle is negative, offset if it is

//   if (angle < 0) {
//     angle += 360;
//   }

//   //Check if boat needs to turn right or left
//   if ((RIGHT_MIN <= angle) && (angle <= RIGHT_MAX))  //turn right
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
//     PWM_change(pwm_value, PWM_MAX_FW);  //TODO: check if right motor should be dampened

//   } else if ((LEFT_MIN <= angle) && (angle <= LEFT_MAX))  //turn left
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
//     PWM_change(PWM_MAX_FW, pwm_value);  //TODO: check if left value needs to be dampened

//   } else if ((angle < RIGHT_MIN) && (angle > LEFT_MIN))  //edge case where boat is backwards
//   {
//     //change right motor to full power and left motor off
//     PWM_change(PWM_STOP, PWM_MAX_FW);
//   } else  //boat is headed within the right direction keep moving forward
//   {
//     PWM_change(PWM_MAX_FW, PWM_MAX_FW);
//   }
// }



// /*
// * Takes in the doa error and then multiples it by the proportional constant.
// * once done it will map the speed to 0-100 range.
// */
// uint32_t proportional_calc(int error) {
//   uint32_t calc_speed;
//   calc_speed = error * P_K;

//   if (calc_speed > 170) {
//     calc_speed = 170;
//   }
//   //remap to power damping range (0 - 100)
//   calc_speed = map(calc_speed, ERROR_MIN, ERROR_MAX, 0, 100);

//   return calc_speed;
// }
// /*
// E-Stop interrupt
// will be connect to pin that reads E-stop signal
// */
// void E_stop() {
//   //set up flags for interrupt
//   static int stop_flag = FALSE;
//   static int re_start = TRUE;
//   //check if the stop flag is high
//   if (stop_flag == FALSE) {
//     //if low then open relays and turn of motors
//     digitalWrite(PB15, 0);  //TODO: get the right pin number

//     //make stop flag high
//     stop_flag = TRUE;
//   }
//   //if stop flag high, check re-start flag
//   else if (re_start == FALSE) {
//     //if it is low then make the flag high
//     re_start = TRUE;
//   } else if ((stop_flag == TRUE) && (re_start == TRUE)) {
//     //close the relays to turn the motors back on
//     digitalWrite(PB15, 1);  //TODO: get the right pin number
//     //re-initalize motors for use
//     motor_esc_init();
//     //reset flags
//     stop_flag = FALSE;
//     re_start = FALSE;
//   }
// }

