#include <Arduino.h>

#define SOUND_SPEED 0.0340 // cm/µs
#define LEDC_TIMER_13_BIT 13
static const long LEDC_BASE_FREQ = 20;  //20Hz
uint32_t dutyCycle = (pow(2, LEDC_TIMER_13_BIT) - 1) / 100; //2kHz

/*Ultrasound 1*/
#define LEDC_CHANNEL_1 0
#define TRIGGER_PIN1 26
#define ECHO_PIN1 25
volatile unsigned long measure1_start = 0;
volatile unsigned long measure1_stop = 0;

/*Ultrasound 2*/
#define LEDC_CHANNEL_2 1
#define TRIGGER_PIN2 19
#define ECHO_PIN2 18
volatile unsigned long measure2_start = 0;
volatile unsigned long measure2_stop = 0;

/*Ultrasound 3*/
#define LEDC_CHANNEL_3 2
#define TRIGGER_PIN3 12
#define ECHO_PIN3 13
volatile unsigned long measure3_start = 0;
volatile unsigned long measure3_stop = 0;

/*Potentiomètre*/
#define TIMER_CHANNEL_4 3
#define POTENTIOMETER_PIN 34
volatile unsigned long angle = 0;
hw_timer_t *Timer3_Cfg = NULL;
//prescaler en fonction de la fréquence de base
static const long freq_base = 80000000;



#define NB_PROCESS 4

struct Distance{
    float distance1 = 0;
    float distance2 = 0;
    float distance3;
    float angle;
};
Distance Dist;


void IRAM_ATTR echo_isr1() {
    if (digitalRead(ECHO_PIN1) == HIGH){
        measure1_start = micros();}
    else{measure1_stop = micros();}
}

void IRAM_ATTR echo_isr2() {
    if (digitalRead(ECHO_PIN2) == HIGH){
        measure2_start = micros();}
    else{
        //Serial.print("received at"); Serial.print(" "); Serial.println(micros());
        measure2_stop = micros();}
}

void IRAM_ATTR echo_isr3() {
    if (digitalRead(ECHO_PIN3) == HIGH){
        measure3_start = micros();}
    else{measure3_stop = micros();}
}

void IRAM_ATTR timer_isr4() {
    angle = analogRead(POTENTIOMETER_PIN);
    Serial.println(angle);
}


void setup() {

    Serial.begin(9600);

    //Setup ultrasound 1
    pinMode(ECHO_PIN1, INPUT);
    attachInterrupt(ECHO_PIN1, echo_isr1, CHANGE);
    ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(TRIGGER_PIN1, LEDC_CHANNEL_1);
    ledcWrite(LEDC_CHANNEL_1, dutyCycle);

    //Setup ultrasound 2 déphasé de 1/4 de période
    delay(1000/LEDC_BASE_FREQ/(NB_PROCESS));
    pinMode(ECHO_PIN2, INPUT);
    attachInterrupt(ECHO_PIN2, echo_isr2, CHANGE);
    ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(TRIGGER_PIN2, LEDC_CHANNEL_2);
    ledcWrite(LEDC_CHANNEL_2, dutyCycle);

    //Setup ultrasound 3 déphasé de 1/4 de période
    delay(1000/LEDC_BASE_FREQ/(NB_PROCESS));
    pinMode(ECHO_PIN3, INPUT);
    attachInterrupt(ECHO_PIN3, echo_isr3, CHANGE);
    ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(TRIGGER_PIN3, LEDC_CHANNEL_3);
    ledcWrite(LEDC_CHANNEL_3, dutyCycle);

    //Setup potentiometer
    /*
    delay(1000/LEDC_BASE_FREQ/(NB_PROCESS+1));
    pinMode(POTENTIOMETER_PIN, INPUT);
    Timer3_Cfg = timerBegin(TIMER_CHANNEL_4, freq_base/LEDC_BASE_FREQ, true);
    timerAttachInterrupt(Timer3_Cfg, &timer_isr4, true);
    timerAlarmWrite(Timer3_Cfg, 100000, true);
    timerAlarmEnable(Timer3_Cfg);
    timerWrite(Timer3_Cfg, 0);    

    */
    delay(1000/LEDC_BASE_FREQ/(NB_PROCESS));


    while(true){
        angle = analogRead(POTENTIOMETER_PIN);

        Dist.distance1 = (measure1_stop - measure1_start)/2 * SOUND_SPEED;
        Dist.distance2 = (measure2_stop - measure2_start)/2 * SOUND_SPEED;
        Dist.distance3 = (measure3_stop - measure3_start)/2 * SOUND_SPEED;
        Dist.angle = angle; //TODO: CALIBRER K*sin(a*angle+b)+c

        Serial.write(reinterpret_cast<const uint8_t*>(&Dist), sizeof(Distance));
        //Serial.print(Dist.distance1);Serial.print("\t");Serial.print(Dist.distance2);Serial.print("\t");Serial.print(Dist.distance3);Serial.print("\t");Serial.println(Dist.angle);
        //Serial.print(measure1_start);Serial.print(" "); Serial.println(measure1_stop);
        delay(1000/LEDC_BASE_FREQ);
    }

}

void loop() {
    // rien à faire ici, le loop décale les mesures
}
