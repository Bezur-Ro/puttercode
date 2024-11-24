/*
		---uses ultrasonic sensor to detect Barn
		---uses gyro for turning
*/


void hole5() {
  // 6 inch
  POW = 50;
  tpow = 50;
  //targetpos = 75;
  gripperclose();
  lft();
  delay(800);
  stp();
  delay(250);
  fwrd();
  delay(1700);
  stp();

  targetpos = 25;
  delay(250);
  rht();
  delay(600);
  stp();
  distance = sr04.Distance();  //get distance in cm
  while (distance >= 15) {     // drive forward until an object is 15cm infront of robot
    fwrd();
  }
  stp();
  targetpos = 30;
  delay(250);
  lft();
  delay(700);
  stp();
  fwrd();
  delay(1800);
  stp();
  rht();
  delay(800);
  stp();
  delay(200);
  fwrd();
  delay(1400);
  stp();

  //targetpos = 55;
  delay(200);
  right(55);
  fwrd();
  delay(500);
  stp();
  gripperopen();
  delay(500);
  rev();
  delay(400);
  stp();
}