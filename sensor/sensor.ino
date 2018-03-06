#define L1 A0
#define F3 A1
#define R2 A2
#define R1 A3
#define F1 A4
#define BACK A5 
#define READ_TIMES 7

int counter = 0;

void setup()
{
  Serial.begin(9600);
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
    sensorVal[i] = analogRead(R1);         
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

