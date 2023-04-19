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

#define THRUSTER_NEUTRAL 1500 // microsec
#define THRUSTER_PWM_PERIOD 4000 //microsec

#define PORT_PIN D10
#define STAR_PIN D9

#define THRUSTER_UPDATE_INTERVAL 

//stuff needed for timers to do PWM
TIM_TypeDef *tim1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PORT_PIN), PinMap_PWM);  //TIM4 channel 3
TIM_TypeDef *tim2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(STAR_PIN), PinMap_PWM);  //TIM4 channel 4 (or at least it should be)
HardwareTimer *port_timer = new HardwareTimer(tim1);  //TIM4 channel 3
HardwareTimer *star_timer = new HardwareTimer(tim2);  //TIM4 channel 4
uint32_t port_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PORT_PIN), PinMap_PWM));
uint32_t star_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(STAR_PIN), PinMap_PWM));

TIM_TypeDef *tim3 = 

uint32_t NextPortValue = 

void setup() {

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

  update_thrusters_timer->attachInterrupt(update_thrusters);

  //motor ESC initializtion sequence
  //motor_esc_init();

  //set up baud rate for UART
  Serial.begin(115200);
}

void loop() {

  //TODO:check to see if this will cause issues. Might change them to static
  
}

void update_thrusters() {
  
  // Setup thruster PWMs
  port_timer->pause();
  star_timer->pause();
  port_timer->setCaptureCompare(port_channel, PortNextValue, MICROSEC_COMPARE_FORMAT);
  star_timer->setCaptureCompare(star_channel, StarNextValue, MICROSEC_COMPARE_FORMAT);
  port_timer->resume();
  star_timer->resume();
}

/*
motor ESC init sequence 
*/
void motor_esc_init() {
  //create 400Hz pwm and give max speed signal
  //port_timer->setPWM(port_channel, port_pin, PWM_FREQ, PWM_MAX_FW);     //TODO:check pins for this //D10
  //star_timer->setPWM(star_channel, star_pin, PWM_FREQ, PWM_MAX_FW);  //D10

  //delay the time needed for motors to recognize change
  delay(1500);

  //give mid frequency stop signal
  //1.5ms pulse width
  port_timer->pause();
  star_timer->pause();

  // port_timer->setPWM(port_channel, port_pin, PWM_FREQ, PWM_STOP);  //TODO:check pins for this //D10
  // star_timer->setPWM(star_channel, star_pin, PWM_FREQ, PWM_STOP);  //D10

  port_timer->resume();
  star_timer->resume();
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

/*
takes in the duty cycle for the left and right motor
and adds in the right delays to set up the PWM
*/
// void PWM_change(uint32_t left_DC, uint32_t right_DC) {
//   port_timer->pause();
//   star_timer->pause();

//   // port_timer->setPWM(left_channel3, pin, PWM_FREQ, left_DC);
//   // star_timer->setPWM(right_channel4, pin2, PWM_FREQ, right_DC);

//   star_timer->resume();
//   star_timer->resume();
// }

/*
Reads serial data
*/


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

