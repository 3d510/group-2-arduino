#define FRONT_RIGHT A0
#define FRONT_CENTER A1
#define FRONT_LEFT A2
#define LEFT A3
#define RIGHT A4
#define BACK A5 
#define READ_TIMES 7

int counter = 0;

void setup()
{
  Serial.begin(115200);
}

void loop() {
//  if (counter == 0)
//    Serial.println("counter,value");
//  if (counter < 1000) {
//    Serial.print(counter);
//    Serial.print(",");
//    Serial.println(analogRead(FRONT_RIGHT));
//    delay(10);
//    counter++;
//  }
  
  int sensorVal[READ_TIMES];
  int sortedSensorVal[READ_TIMES];
  for (int i = 0; i < READ_TIMES; i++) {
    sensorVal[i] = analogRead(FRONT_LEFT);         
    delay(10);
  }
  sortedSensorVal[0] = sensorVal[0];
  for (int i = 1; i < READ_TIMES; i++) {
    int id = 0;
    while (id < i && sortedSensorVal[id] <= sensorVal[i])
      id++;
    if (id == i) {
      sortedSensorVal[i] = sensorVal[i];
    } else {
      for (int j = i - 1; j >= id; j--) {
        sortedSensorVal[j+1] = sensorVal[j]; 
      } 
      sortedSensorVal[id] = sensorVal[i]; 
    }
  }
  if (counter == 0) {
    Serial.println("count,value");
  }
  if (counter < 1000 && counter >= 0) {
    Serial.print(counter);
    Serial.print(",");
    Serial.println(sortedSensorVal[READ_TIMES/2]);
  }
  counter++;
}

