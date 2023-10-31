#define BUTTON_PINA 5
#define BUTTON_PINB 6
#define BUTTON_PINC 7
#define LED_RED 8
#define LED_YELLOW 9
#define LED_GREEN 10
int systemState = 0;    // Variable to store the button state
int receivedData = 0;

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(BUTTON_PINA, INPUT_PULLUP);
  pinMode(BUTTON_PINB, INPUT_PULLUP);
  pinMode(BUTTON_PINC, INPUT_PULLUP);
  Serial.begin(9600); //Initialise the serial communication
}

void loop() {
  switch (systemState){
    case 0:
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_GREEN, LOW);
      if (digitalRead(BUTTON_PINA) == LOW) {
        systemState = 1;
      }
      else if (Serial.available() > 0) {
        receivedData = Serial.parseInt();
        if (receivedData == 4){
          systemState = 1;
        }
      }
      break;
    case 1: //e-stop is pressed
      digitalWrite(LED_RED, HIGH);
      if (digitalRead(BUTTON_PINB) == LOW) {
        systemState = 2;
      }
      else if (Serial.available() > 0) {
        receivedData = Serial.parseInt();
        if (receivedData == 5){
          systemState = 2;
        }
      }
      break;
    case 2: //e-stop release is pressed
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_YELLOW, HIGH);

      if (digitalRead(BUTTON_PINC) == LOW) {
        systemState = 3;
      }
      else if (Serial.available() > 0) {
        receivedData = Serial.parseInt();
        if (receivedData == 6){
          systemState = 3;
        }
      }
      break;
    case 3: //e-stop is released. MATLAB needs to confirm
      if (Serial.available() > 0) {
        receivedData = Serial.parseInt();
//        Serial.print("Received: ");
//        Serial.println(receivedData);
        if (receivedData == 7){
          digitalWrite(LED_GREEN, HIGH);
          systemState = 0;
        }
      }
      break;
    default:
      break;  
  }
  Serial.println(systemState);          // Send the state to MATLAB via serial
  delay(100);
}
