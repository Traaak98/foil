/**
#include <Arduino.h>

<<<<<<< Updated upstream
#define TRIGGER_PIN 18
#define ECHO_PIN    19
#define SOUND_SPEED 0.00034 // mm/µs
 
const unsigned long MEASURE_TIMEOUT = 2500000UL; // 25ms = ~8m à 340m/s



void setup() {
   
  Serial.begin(9600);
   
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  Serial.println("Triggered");
  
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT); // en mi
  float distance_mm = measure / 2.0 * SOUND_SPEED;
   
  Serial.print(F(">Distance:"));
  Serial.print(distance_mm);
  Serial.println(F("|mm"));
   
  delay(100);
}
*/

#include <Arduino.h>
=======
#define TRIGGER_PIN 19
#define ECHO_PIN    18
#define SOUND_SPEED 0.34 // mm/µs
 
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
>>>>>>> Stashed changes

#define SOUND_SPEED 0.000340 // m/µs
#define TRIGGER_PIN2 26
#define ECHO_PIN2    25
float dist1[3] = {0};
#define TRIGGER_PIN1 19
#define ECHO_PIN1    18
float dist2[3] = {0};

long measure;
 
struct Distance{
    float distance1 = 0;
    float distance2 = 0;
    float distance3 = 0;
    float angle     = 0;
};
Distance Dist;


void setup() {
   
  Serial.begin(9600);
   
<<<<<<< Updated upstream
  pinMode(TRIGGER_PIN1, OUTPUT);
  pinMode(TRIGGER_PIN2, OUTPUT);
  digitalWrite(TRIGGER_PIN1, LOW);
  digitalWrite(TRIGGER_PIN2, LOW);

  pinMode(ECHO_PIN1, INPUT);
  pinMode(ECHO_PIN2, INPUT);
}

void loop() {

	for (int i = 0; i <2; i++){ 
	    dist1[i] = dist1[i+1];
	    dist2[i] = dist2[i+1];
	}
  
  digitalWrite(TRIGGER_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN1, LOW);
  measure = pulseIn(ECHO_PIN1, HIGH, 250000UL);
  dist1[2] = measure / 2.0 * SOUND_SPEED;
  if (dist1[2] > 2){ dist1[2] = dist1[1];}

  digitalWrite(TRIGGER_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN2, LOW); 
  measure = pulseIn(ECHO_PIN2, HIGH, 25000UL); 
  dist2[2] = measure / 2.0 * SOUND_SPEED;
  if (dist2[2] > 2){ dist2[2] = dist2[1];}


	//moyenne des mesures
	Dist.distance1 = 0.;
	Dist.distance2 = 0.;
	Dist.distance3 = 0.;
	Dist.angle     = 0.;

	for (int i = 0; i < 3; i++){
	    Dist.distance1 += dist1[i];
	    Dist.distance2 += dist2[i];
	}
	Dist.distance1 =  Dist.distance1/3;
	Dist.distance2 =  Dist.distance2/3;
  Dist.distance3 =  Dist.distance3;
  Dist.angle =  Dist.angle;

   
  //Serial.print(Dist.distance1);Serial.print("\t");
  //Serial.print(Dist.distance2);Serial.print("\t");
  //Serial.print(Dist.distance3);Serial.print("\t");
  //Serial.println(Dist.angle);
  //while(Serial.available() < sizeof(Distance)){
  //  delay(10);
  //}
  Serial.write(reinterpret_cast<const u_int8_t*>(&Dist), sizeof(Distance));
   
  delay(50);
}
=======
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT); // en mi
  float distance_mm = measure / 2.0 * SOUND_SPEED;
   
  Serial.print(F(">Distance:"));
  Serial.print(distance_mm);
  Serial.println(F("|mm"));
   
  delay(100);
}
>>>>>>> Stashed changes
