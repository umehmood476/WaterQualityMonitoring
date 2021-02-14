
#include <SoftwareSerial.h>
SoftwareSerial snport(D1,D2);

struct data_var {
  float var1 =0.0;
  float var2 =0.0;
  float var3=0.0;
  float var4=0.0;
  
  
  
  };
  data_var var;

  byte buf[sizeof(var)];
void setup() {
  // put your setup code here, to run once:
  snport.begin(9600);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Data received and size of structure is ");
  Serial.print(sizeof(var));

  if(snport.available()>0){

   

     
      snport.readBytes(buf , sizeof(buf));
      
      memcpy(&var , buf , sizeof(buf));
      
      Serial.print("var1: ");
      Serial.println(var.var1);
      Serial.print("var2: ");
      Serial.println(var.var2);
      Serial.print("var3: ");
      Serial.println(var.var3);
      Serial.print("var4: ");
      Serial.println(var.var4);
      
    //Serial.print("Data is ");
    //Serial.println(b);
    
    }
else {
  Serial.print("Data not received and size of structure is ");
  Serial.print(sizeof(var));

  
  }
  delay(1000);
}
