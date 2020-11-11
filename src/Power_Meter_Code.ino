//POWER METER CODE

const int vPIN = A0;      //Arduino Analog Pin



float vOUT = 0.0;

float vIN = 0.0;



float R1 = 1300.0;       //Resistor 1 value in ohms

float R2 = 5100.0;        //Resistor 2 value in ohms



int value = 0;



void setup()

{

  Serial.begin(9600);

  delay(2000);

}



void loop()

{

  value = analogRead(vPIN);

  vOUT = (value * 5.0) / 1024.0;

  

  vIN = vOUT / ( R2 / (R1 + R2) );

  

  Serial.print("Voltage: ");

  Serial.print(vIN);

  Serial.println("V");

    float amp = analogRead(A1);
    amp = amp - 510; 
    amp = (amp * 5.0) / 1024.0; 
    amp = amp * 10; 
    amp = amp;
    Serial.print("A: ");
    Serial.println(amp,1);

        float watt = amp * vIN; 
    Serial.print("W: ");
    Serial.println(watt,1);

  delay(500);



}
