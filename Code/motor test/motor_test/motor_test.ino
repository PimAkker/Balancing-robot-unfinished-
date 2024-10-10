int motor1pin1 = A2;
int motor1pin2 = A3;

int motor2pin1 = 4;
int motor2pin2 = 5;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
 
  pinMode(11, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:   

  //Controlling speed (0 = off and 255 = max speed):
  //analogWrite(11, 100); //ENA pin
  for (int i = 0; i < 255; i++)
{
 

  analogWrite(11, i); //ENB pin

  //Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  Serial.print("speed: ");
  Serial.println(i);
  
  delay(100);
}

}
