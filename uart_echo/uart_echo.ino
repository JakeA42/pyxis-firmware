#define TRUE 1
#define FALSE 0
#define MSG_BUFF_SIZE 24
#define MAX_MSG_LEN (MSG_BUFF_SIZE - 1)

#define pin D10
#define pin2 D9

//attach serial to D0 (Rx) and D1 (tx)
HardwareSerial Serial1(D0, D1);

int NewMessage = FALSE;

void setup() {

  //set up baud rate for UART
  Serial1.begin(115200);
  Serial.begin(115200);

  delay(5000);
  Serial.println("Heyo");
}

void loop() {
  static bool isfullmsg = FALSE;
  //TODO:check to see if this will cause issues. Might change them to static
  static char msg[MSG_BUFF_SIZE] = {0};
  static uint8_t msglen = 0;  
  
  //call the UART function to get all of the info needed
  msglen = UART(msg, msglen);
  if (NewMessage)
  {
    Serial.println(msg);
    NewMessage = FALSE;
    msglen = 0;
  }

}

/*
Reads serial data
*/
int UART(char recv_data[], uint8_t msglen) {

  while (Serial1.available() && msglen < MAX_MSG_LEN)
  {
    recv_data[msglen] = Serial1.read();

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
