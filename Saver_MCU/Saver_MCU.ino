#define NUM_BYTES     8
#define RIGHT_MOT     4
#define LEFT_MOT      3
#define PWM_FREQ      400
#define PWM_MAX_FW    80
#define PWM_MAX_RV    40 
#define PWM_STOP      60 
#define PWR_BYTE_NUM  3 //TODO: need to check this
#define PWR_THRESH    10 //TODO: need to figure out what this will be
#define DOA_BYTE_NUM  0
#define P_K           1
#define LEFT_MIN      185
#define LEFT_MAX      355
#define RIGHT_MAX     175
#define RIGHT_MIN     5
#define ERROR_MAX     170
#define ERROR_MIN     0

typedef enum STATE_TYPE
{
  GET_DATA,
  SPD_ADJ,
  MOV_HOLD
}STATE_TYPE;

//attach serial to D0 (Rx) and D1 (tx)
HardwareSerial Serial1(D0, D1);

//set up timer 4 for use
HardwareTimer  TIM4_PWM(TIM4);

//starting state for the state machine
//didn't want to make it global but arduino is stupid
STATE_TYPE state = GET_DATA;

void setup() {
  //turn on pi
  //set up gpio pin as output
  pinMode(PA5, OUTPUT); //A1
  //send signal to close relay
  digitalWrite(PA5,1);
    
  //turn on kraken
  //set up gpio pin as output
  pinMode(PA6, OUTPUT);    
  //send signal to close relay
  digitalWrite(PA6,1);
    
  
  //motor ESC initializtion sequence
  motor_esc_init();

  //E-stop interrupt initalization sequence?

  //TODO: check if there is a time delay needed before taking readings from kraken
  //set up baud rate for UART
  Serial1.begin(115200);


}

void loop() {
  
  //TODO:check to see if this will cause issues. Might change them to static
  u_char data[NUM_BYTES];
  u_char power_data = 0;
  uint32_t doa = 0;
  uint32_t left_DC = 0;
  uint32_t right_DC = 0;

  switch(state)
  {
    case GET_DATA:
      //call the UART function to get all of the info needed
      UART(data);

      //check the power data and see if it should move or stay put
      power_data = data[PWR_BYTE_NUM];//TODO: Figure out what bytes will hold power info
      
      //move to the next state
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
      //get doa data
      doa = data[DOA_BYTE_NUM];   
      //calculate and set motor speeds      
      pwm_calc(doa);
      //go to get_data state
      state = GET_DATA;
    break;

    case MOV_HOLD:
      //change timer count to neutral timing
      TIM4_PWM.setPWM(LEFT_MOT, D8, PWM_FREQ, PWM_STOP);
      TIM4_PWM.setPWM(RIGHT_MOT, D9, PWM_FREQ, PWM_STOP);
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
  //create 50Hz pwm and give max speed signal
  TIM4_PWM.setPWM(LEFT_MOT, D8, PWM_FREQ, PWM_MAX_FW); //TODO:check pins for this
  TIM4_PWM.setPWM(RIGHT_MOT, D9, PWM_FREQ, PWM_MAX_FW);
  
  //give mid frequency stop signal
    //1.5ms pulse width
  TIM4_PWM.setPWM(LEFT_MOT, D8, PWM_FREQ, PWM_STOP);
  TIM4_PWM.setPWM(RIGHT_MOT, D9, PWM_FREQ, PWM_STOP);
}

/*
PWM signal calculation function
calculates the counter value for the capture compare channels then sets it
*/
void pwm_calc(int doa)
{
  //variables for calculating proportional speed
  int error = 0;
  int calc_speed = 0;
  
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
    //make right motors full power
    TIM4_PWM.setPWM(RIGHT_MOT, D8, PWM_FREQ, PWM_MAX_FW);
    
    //calculate the error
    error = RIGHT_MAX - angle;

    calc_speed = proportional_calc(error);
    
    //set the new value
    TIM4_PWM.setPWM(RIGHT_MOT, D8, PWM_FREQ, calc_speed);
  }  
  else if((LEFT_MIN <= angle) && (angle <= LEFT_MAX)) //turn left
  {
    //make left motor full power
    TIM4_PWM.setPWM(LEFT_MOT, D8, PWM_FREQ, PWM_MAX_FW);

    //compute error
    error = LEFT_MAX - angle;

    calc_speed = proportional_calc(error);
      
    TIM4_PWM.setPWM(RIGHT_MOT, D8, PWM_FREQ, calc_speed);
  }
  else if((angle < RIGHT_MIN) && (angle > LEFT_MIN))//edge case where boat is backwards
  {
    //turn right
    TIM4_PWM.setPWM(RIGHT_MOT, D8, PWM_FREQ, PWM_MAX_FW);
    //turn left motor off
    TIM4_PWM.setPWM(LEFT_MOT, D8, PWM_FREQ, PWM_STOP);    
  }
  else //boat is head within the right direction keep moving forward
  {
    TIM4_PWM.setPWM(LEFT_MOT, D8, PWM_FREQ, PWM_MAX_FW); 
    TIM4_PWM.setPWM(RIGHT_MOT, D9, PWM_FREQ, PWM_MAX_FW);
  }
}

/*
Reads serial data
*/
void UART(u_char recv_data[])
{
  //TODO: figure out how many bytes we need

  //counter to keep track of how many bytes have been recieved
  uint8_t counter = 0;

  while(counter != NUM_BYTES)
  {
    if(Serial1.available())
    {
      //Read a byte from the Serial buffer
      recv_data[counter] = Serial1.read();
      //increment counter
      counter++;
    }
  }
}

uint32_t proportional_calc(int error)
{
  uint32_t calc_speed;
  calc_speed = error * P_K;    

  if(calc_speed > 170)
  {
    calc_speed = 170;
  }
  //remap to PWM ranges (80-60)
  calc_speed = map(calc_speed, ERROR_MIN, ERROR_MAX, PWM_STOP, PWM_MAX_FW);

  return calc_speed;
}
/*
E-Stop interrupt
will be connect to pin that reads E-stop signal
*/
// ISR()
// {

// }









