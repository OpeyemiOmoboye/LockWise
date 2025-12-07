//we will use the pushbutton as a standin for the mc-38
#include <Arduino.h>
#include <Timers.h>
#include <ESP32Servo.h>
Servo servo;

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;


//pin wiring
const int servoPin = 2;
const int REED_PIN = 4;
const int INSIDE_PIR_PIN = 15;
const int OUTSIDE_PIR_PIN = 18;

//states and events
typedef enum {LOCKED_STATE, UNLOCKED_STATE, DOOR_OPEN_STATE, DOOR_CLOSED_STATE} system_state_t;
typedef enum{EVENT_INSIDE_MOTION, EVENT_OUTSIDE_MOTION, EVENT_DOOR_OPEN, EVENT_DOOR_CLOSED, EVENT_LOCK_TIMER_EXPIRES} system_event_t;

//rtos objects
QueueHandle_t xEventQueue;
SemaphoreHandle_t xStateMutex;
TimerHandle_t xDoorClosedStateTimer, xDoorOtherStatesTimer;
volatile system_state_t currentState = LOCKED_STATE;

//timer setup and function
const int door_timer_closed_state = 5000;
const int door_timer_open_unlocked_states = 10000;
void timercallback(TimerHandle_t){
  system_event_t e = EVENT_LOCK_TIMER_EXPIRES;
  xQueueSend(xEventQueue, &e, 0);
}


void servoRotate(system_state_t state){
  if(state == UNLOCKED_STATE){
      for (int pos = 0; pos <= 90; pos += 1) {
        servo.write(pos);
        delay(15);
      }
  }
  else if(state == LOCKED_STATE){
      for (int pos = 90; pos >= 0; pos -= 1) {
        servo.write(pos);
        delay(15);
      }
  }
}


void sendMessage(const char* msg) {
  Serial.println(msg);
  SerialBT.println(msg);
}


void stateMachineTask(void *){
  system_event_t e;
  for(;;){
    if(xQueueReceive(xEventQueue,&e,portMAX_DELAY)==pdTRUE){
      xSemaphoreTake(xStateMutex,portMAX_DELAY);
      system_state_t old =currentState;

      if(currentState == LOCKED_STATE){
        if(e == EVENT_INSIDE_MOTION) {
          currentState = UNLOCKED_STATE;
          xTimerStart(xDoorOtherStatesTimer, door_timer_open_unlocked_states);
          sendMessage("Inside Motion: LOCKED -> UNLOCKED");
          servoRotate(currentState);
          }
        else if(e == EVENT_OUTSIDE_MOTION) {
          sendMessage("Outside Motion: LOCKED");
          
          }
      }

      else if(currentState == UNLOCKED_STATE){
        if(e == EVENT_DOOR_OPEN){
          currentState = DOOR_OPEN_STATE;
          xTimerStart(xDoorOtherStatesTimer, door_timer_open_unlocked_states);
          sendMessage("Door Opened: UNLOCKED -> DOOR OPEN");
        }
        else if(e == EVENT_LOCK_TIMER_EXPIRES){
          currentState = LOCKED_STATE;
          sendMessage("UNLOCKED -> LOCKED");
          servoRotate(currentState);
        }
      }

      else if(currentState == DOOR_OPEN_STATE){
        if(e == EVENT_DOOR_CLOSED){
          currentState = DOOR_CLOSED_STATE;
          xTimerStart(xDoorOtherStatesTimer, door_timer_closed_state);
          sendMessage("Door Closed: DOOR OPEN -> DOOR CLOSED");
        }
        else if(e == EVENT_LOCK_TIMER_EXPIRES){
          xTimerStart(xDoorOtherStatesTimer, door_timer_closed_state);
          sendMessage("Door is still open");
        }
      }

      else if(currentState == DOOR_CLOSED_STATE && (e == EVENT_LOCK_TIMER_EXPIRES || e == EVENT_OUTSIDE_MOTION)){
        currentState = LOCKED_STATE;
        if(e == EVENT_LOCK_TIMER_EXPIRES){sendMessage("DOOR CLOSED -> LOCKED");}
        else{sendMessage("Outside Motion: DOOR CLOSED -> LOCKED");}
        servoRotate(currentState);
      }

    xSemaphoreGive(xStateMutex);
    }
  }
}

struct Sensor{
  int pin;
  bool lastStable, lastRead;
  uint32_t lastChange, lastFire;
};
Sensor door_reed = {REED_PIN, HIGH, HIGH, 0, 0};
Sensor inside_pir = {INSIDE_PIR_PIN, LOW, LOW, 0, 0};
Sensor outside_pir = {OUTSIDE_PIR_PIN, LOW, LOW, 0, 0};
Sensor sensors[] = {door_reed, inside_pir, outside_pir};
const unsigned long DEBOUNCE_DELAY = 30;
const unsigned long LONG_PRESS_DELAY = 150;

void sensorReadTask(void *){
  for(;;){
    uint32_t currentTime = millis();

    for(auto &sensor: sensors){
      system_event_t e;
      int sensor_reading = digitalRead(sensor.pin);
      if(sensor_reading != sensor.lastRead){
        if(sensor.pin == INSIDE_PIR_PIN && sensor_reading){e = EVENT_INSIDE_MOTION; xQueueSend(xEventQueue, &e, 0);}
        else if(sensor.pin == OUTSIDE_PIR_PIN && sensor_reading){e = EVENT_OUTSIDE_MOTION; xQueueSend(xEventQueue, &e, 0);}
        sensor.lastChange = millis(); //for reed debouncing
        sensor.lastRead = sensor_reading;
      }

      if(sensor.pin == REED_PIN){
        
        if(millis() - sensor.lastChange > DEBOUNCE_DELAY){
          if(sensor_reading != sensor.lastStable){
            sensor.lastStable = sensor_reading;
            if(millis() - sensor.lastFire > LONG_PRESS_DELAY){
              sensor.lastFire = millis();
              if(!digitalRead(REED_PIN)){
                e = EVENT_DOOR_CLOSED;
              }
              else{
                e = EVENT_DOOR_OPEN;
                }
              xQueueSend(xEventQueue, &e, 0);
            }
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));    
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("DoorSystem");

  //pin set up
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(INSIDE_PIR_PIN, INPUT);
  pinMode(OUTSIDE_PIR_PIN, INPUT);
  Serial.println("LOCKED");
  SerialBT.println("LOCKED");


  servo.setPeriodHertz(50);
  servo.attach(servoPin, 500, 2400);
  servo.write(0);

  

  //object creations
  xEventQueue = xQueueCreate(10, sizeof(system_event_t));
  xStateMutex = xSemaphoreCreateMutex();
  //timer creation
  xDoorClosedStateTimer = xTimerCreate("Closed State Timer", pdMS_TO_TICKS(door_timer_closed_state), pdFALSE, NULL, timercallback);
  xDoorOtherStatesTimer = xTimerCreate("Other State Timer", pdMS_TO_TICKS(door_timer_open_unlocked_states), pdFALSE, NULL, timercallback);
  //pin
  xTaskCreatePinnedToCore(stateMachineTask, "State Machine", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(sensorReadTask, "Sensors", 4096, NULL, 1, NULL, 1);
}

void loop() {

}

