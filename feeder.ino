#define PIN_RELAY_1  D1 // The ESP8266 pin connected to the IN1 pin of relay module
#define PIN_RELAY_2  D2 // The ESP8266 pin connected to the IN2 pin of relay module
int val;
const int dirPin = D5;
const int stepPin = D4;
const int trigPin = 12;
const int echoPin = 13;
const int stepsPerRevolution = 300;
const int delayTime = 500; // in microseconds, adjust this to change speed
 
void setup() {
  // Declare pins as Outputs    
  Serial.begin(115200);
  pinMode(PIN_RELAY_1, OUTPUT); //motor
  pinMode(PIN_RELAY_2, OUTPUT); //stepper motor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
} 
void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.0343 / 2;
if (distance <= 2)
  Serial.println("FULL");
  else if (distance >= 25)
   Serial.println("EMPTY");
delay(1000);
  digitalWrite(PIN_RELAY_1, HIGH);
  digitalWrite(PIN_RELAY_2, HIGH);
  delay(5000);
  digitalWrite(PIN_RELAY_1, LOW);
  digitalWrite(PIN_RELAY_2, LOW);
  delay(5000);
  clockwiseRotation();
  delay(300);
  counterclockwiseRotation();
  delay(300);
}
void clockwiseRotation() {
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);
 
  // Spin motor at a constant speed
  for(int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);
  }
}
void counterclockwiseRotation() {
  // Set motor direction counterclockwise
  digitalWrite(dirPin, LOW); 
  // Spin motor at a constant speed
  for(int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);
  }
}