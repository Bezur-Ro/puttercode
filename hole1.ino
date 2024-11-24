/*
		--- On This Hole Putters is dedicated to promoting safer work environments. 
    They utilize the MPU6050 temperature sensor to ensure driving operations are conducted only when the temperature ranges between 68-76°F, 
    in accordance with OSHA guidelines.
     This commitment helps maintain optimal working conditions and prioritize employee safety.
*/
void hole1() {
  output();
  if (temp > 68 && temp < 76) {
    POW = 67;
    delay(500);
    gripperclose();
    fwrd();
    delay(2500);
    stp();
    delay(300);
    gripperopen();
    delay(300);
    rev();
    delay(1000);
    stp();
  }
}