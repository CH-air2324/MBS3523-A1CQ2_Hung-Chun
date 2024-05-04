#include <Servo.h> 
  

Servo LR_; 

Servo UD_; 

void setup() { 

  LR_.attach(3); 

  UD_.attach(2); 

  Serial.begin(115200); 

} 

void loop() { 

  while(Serial.available()) { 

    String inputString = Serial.readStringUntil('\r'); 

    int commaIndex = inputString.indexOf(','); 

    if (commaIndex != -1) { 

      int x_axis = inputString.substring(0, commaIndex).toInt(); 

      int y_axis = inputString.substring(commaIndex + 1).toInt(); 
  

      int y = map(y_axis, 0, 1080, 0, 180); 

      int x = map(x_axis, 0, 1920, 0, 180); 

      LR_.write(x); 

      UD_.write(y); 

      // Print the parsed values 

      Serial.print("First Integer: "); 

      Serial.println(x); 

      Serial.print("Second Integer: "); 

      Serial.println(y); 

    } 

  } 

} 