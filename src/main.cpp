#include <Arduino.h>
#include <StateMachine.h>
#include <State.h>
#include <LinkedList.h>

const int STATE_DELAY = 1000;
int randomState = 0;
const int LED = 13;

StateMachine machine = StateMachine();
void state0();

State* S0 = machine.addState(&state0); 


void setup() {
    Serial.begin(115200);
  pinMode(LED,OUTPUT);

}

void loop() {
    machine.run();
    delay(STATE_DELAY);
}

void state0(){
  Serial.println("State 0");
  if(machine.executeOnce){
    Serial.println("Execute Once");
    digitalWrite(LED,!digitalRead(LED));
  }
}