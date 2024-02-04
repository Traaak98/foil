#include <Arduino.h>

#define SOUND_SPEED 0.0340 // cm/µs
#define LEDC_TIMER_13_BIT 13
#define LEDC_BASE_FREQ 1    //10Hz
uint32_t dutyCycle = (pow(2, LEDC_TIMER_13_BIT) - 1) / 1000; //1kHz


/*Ultrasound 0*/
#define LEDC_CHANNEL_0 0
#define TRIGGER_PIN0 26
#define ECHO_PIN0 25

volatile unsigned long measure0_start = 0;
volatile unsigned long measure0_stop = 0;
float distance0_cm = 0;


void IRAM_ATTR echo_isr0() {
    if (digitalRead(ECHO_PIN0) == HIGH){
        measure0_start = micros();}
    else{measure0_stop = micros();}
}




void setup() {

    Serial.begin(9600);

    pinMode(ECHO_PIN0, INPUT);
    attachInterrupt(ECHO_PIN0, echo_isr0, CHANGE);
    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(TRIGGER_PIN0, LEDC_CHANNEL_0);
    ledcWrite(LEDC_CHANNEL_0, dutyCycle);

    
}

void loop() {

    distance0_cm = (measure0_stop - measure0_start)/2 * SOUND_SPEED;
    Serial.print(">Distance0:"); Serial.println(distance0_cm);
    
    delay(1000);

}