
#define NUM_BYTES 8

typedef enum STATE_TYPE
{
  GET_DATA,
  SPD_ADJ,
  MOV_HOLD
}STATE_TYPE;

//attach serial to D0 (Rx) and D1 (tx)
HardwareSerial Serial1(D0, D1);

//starting state for the state machine
//didn't want to make it global but arduino is stupid
STATE_TYPE state = GET_DATA;

void setup() {
  // put your setup code here, to run once:

  //set up baud rate for UART
  Serial.begin(115200);


}

void loop() {
  // put your main code here, to run repeatedly:
  switch(state)
  {
    case GET_DATA:
    break;      

    case SPD_ADJ:
    break;

    case MOV_HOLD:
    break;    
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