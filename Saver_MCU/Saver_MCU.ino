#define TRUE          1
#define FALSE         0
#define NUM_BYTES     20
#define RIGHT_MOT     4
#define LEFT_MOT      3
#define PWM_FREQ      400
#define PWM_MAX_FW    80
#define PWM_MAX_RV    40 
#define PWM_STOP      60 
#define PWR_THRESH    20 
#define P_K           1
#define LEFT_MIN      185
#define LEFT_MAX      355
#define RIGHT_MAX     175
#define RIGHT_MIN     5
#define ERROR_MAX     170
#define ERROR_MIN     0

#define pin           D10
#define pin2          D9
#define PI_PIN        A5
#define KRAKEN_PIN    A4
#define E_STOP_PIN    D6
#define MOTOR_PIN     PB15  //COPI pin
#define STANDBY_PIN   D11
#define START_PIN     D12

typedef enum STATE_TYPE
{
  GET_DATA,
  SPD_ADJ,
  MOV_HOLD
}STATE_TYPE;

//stuff needed for timers to do PWM
TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM); //TIM4 channel 3
TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin2), PinMap_PWM); //TIM4 channel 4 (or at least it should be)

HardwareTimer *left_timer = new HardwareTimer(Instance); //TIM4 channel 3
HardwareTimer *right_timer = new HardwareTimer(Instance2); //TIM4 channel 4

uint32_t left_channel3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM)); 
uint32_t right_channel4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin2), PinMap_PWM));

//attach serial to D0 (Rx) and D1 (tx)
HardwareSerial Serial1(D0, D1);

//starting state for the state machine
//didn't want to make it global but arduino is stupid
STATE_TYPE state = GET_DATA;

int standby_flag = 0;
int start_flag = 0;

void setup() {
  
  //set up motor relay pin
  pinMode(MOTOR_PIN, OUTPUT); 
  
  //set up pi relay pin
  pinMode(PI_PIN, OUTPUT); 
    
  //set up kraken pin
  pinMode(KRAKEN_PIN, OUTPUT);    

  pinMode(STANDBY_PIN, INPUT_PULLUP);

  pinMode(START_PIN, INPUT_PULLUP);
  
  //E-stop interrupt initalization 
  pinMode(E_STOP_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN),E_stop,FALLING);

  //set up baud rate for UART
  Serial1.begin(9600);
  Serial.begin(9600);


}

void loop() {
  
  //TODO:check to see if this will cause issues. Might change them to static
  char data[NUM_BYTES];
   
  static int32_t power_data = 0; 
  static uint32_t doa = 0;
  static uint32_t conf = 0;  
  
  Serial.println(digitalRead(STANDBY_PIN));
  Serial.println("test");

  if((digitalRead(STANDBY_PIN) == 0) && (standby_flag == 0))
  {
    standby_flag = 1;
    //turn on raspberry pi
    digitalWrite(PI_PIN,1); 

    //turn on kraken
    digitalWrite(KRAKEN_PIN,1);

    Serial.println("standby done");

  }

  if((digitalRead(START_PIN) == 0) && (start_flag == 0))
  {
    start_flag = 1;
    //turn on motors
    digitalWrite(MOTOR_PIN,1);
    //initialize motors
    motor_esc_init();

    Serial.println("start done");
  }

  switch(state)
  {
    case GET_DATA:
      //call the UART function to get all of the info needed
      UART(data);

      //Break recieved data up and convert to numerical values
      sscanf(data, "%i,%i,%i", &doa, &conf, &power_data);

      //check power data to determine which state to go to
      if(power_data > PWR_THRESH)
      {
        state = MOV_HOLD;       
      }
      else
      {
        state = SPD_ADJ;
      }
    break;      

    case SPD_ADJ: 
      //calculate and set motor speeds      
      pwm_calc(doa);
      //go to get_data state
      state = GET_DATA;
    break;

    case MOV_HOLD:
      //change timer count to neutral timing
      PWM_change(PWM_STOP, PWM_STOP);

      //go back to GET_DATA
      state = GET_DATA;
      
    break;    
  } 
}

/*
motor ESC init sequence 
*/
void motor_esc_init()
{
  //create 400Hz pwm and give max speed signal
  left_timer->setPWM(left_channel3, pin, PWM_FREQ, PWM_MAX_FW); //TODO:check pins for this //D10 
  right_timer->setPWM(right_channel4, pin2, PWM_FREQ, PWM_MAX_FW); //D10
  
  //delay the time needed for motors to recognize change
  delay(1500);

  //give mid frequency stop signal
  //1.5ms pulse width
  left_timer->pause();
  right_timer->pause();

  left_timer->setPWM(left_channel3, pin, PWM_FREQ, PWM_STOP); //TODO:check pins for this //D10 
  right_timer->setPWM(right_channel4, pin2, PWM_FREQ, PWM_STOP); //D10

  left_timer->resume();
  right_timer->resume();
}

/*
PWM signal calculation function
calculates the counter value for the capture compare channels then sets it
*/
void pwm_calc(uint32_t doa)
{
  //variables for calculating proportional speed
  int error = 0;
  int calc_speed = 0;
  uint32_t pwm_value = 0;
  
  //remap doa angle so 45 = 0
  int angle = doa - 45;
  //check if the angle is negative, offset if it is

  if(angle < 0)
  {
    angle += 360;
  }

  //Check if boat needs to turn right or left
  if((RIGHT_MIN <= angle) && (angle <= RIGHT_MAX))  //turn right
  {
    //calculate the error
    error = angle;

    //get the proportional speed
    calc_speed = proportional_calc(error);

    //damping the power on an exponential curve
    //TODO: either take out or change calculation
    //calc_speed = sqrt(calc_speed);  

    //remap power level to pwm duty cycle range (60 - 80)
    pwm_value = map(calc_speed, 0, 100, PWM_STOP, PWM_MAX_FW);
  
    //change PWM values
    PWM_change(pwm_value,PWM_MAX_FW); //TODO: check if right motor should be dampened

  }  
  else if((LEFT_MIN <= angle) && (angle <= LEFT_MAX)) //turn left
  {
    //compute error
    error = LEFT_MAX - angle;
    
    //get the calculated speed vvalue
    calc_speed = proportional_calc(error);

    //damping the power on an exponential curve
    //TODO: either take out or change calculation
    //calc_speed = sqrt(calc_speed); 

    //remap power level to pwm duty cycle range (60 - 80)
    pwm_value = map(calc_speed, 0, 100, PWM_STOP, PWM_MAX_FW);
  
    //change duty cycles of the left and right motor
    PWM_change(PWM_MAX_FW, pwm_value);  //TODO: check if left value needs to be dampened

  }
  else if((angle < RIGHT_MIN) && (angle > LEFT_MIN))//edge case where boat is backwards
  {
    //change right motor to full power and left motor off
    PWM_change(PWM_STOP, PWM_MAX_FW);     
  }
  else //boat is headed within the right direction keep moving forward
  {
    PWM_change(PWM_MAX_FW,PWM_MAX_FW);  
  }
}

/*
takes in the duty cycle for the left and right motor
and adds in the right delays to set up the PWM
*/
void PWM_change(uint32_t left_DC, uint32_t right_DC)
{
  left_timer->pause();
  right_timer->pause();
  
  left_timer->setPWM(left_channel3, pin, PWM_FREQ, left_DC); 
  right_timer->setPWM(right_channel4, pin2, PWM_FREQ, right_DC);

  left_timer->resume();
  right_timer->resume();
}

/*
Reads serial data
*/
void UART(char recv_data[])
{
  //counter to keep track of how many bytes have been recieved
  uint8_t counter = 0;

  while(counter != NUM_BYTES)
  {
    if(Serial1.available())
    {
      //Read a byte from the Serial buffer
      recv_data[counter] = Serial1.read();

      //check for newline character
      if(recv_data[counter] == '\n')
        break;
      
      //increment counter
      counter++;
    }
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
  static int re_start = TRUE;
  //check if the stop flag is high
  if(stop_flag == FALSE)
  {
    //if low then open relays and turn of motors
    digitalWrite(MOTOR_PIN,0); //TODO: get the right pin number

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
    //close the relays to turn the motors back on
    digitalWrite(MOTOR_PIN,1); //TODO: get the right pin number 
    //re-initalize motors for use
    motor_esc_init();       
    //reset flags
    stop_flag = FALSE;
    re_start = FALSE;
  }
}



