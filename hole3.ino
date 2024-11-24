/*
		---uses gyro for turning
*/


void hole3() {

  POW = 50;
  tpow = 50;
 // targetpos = 85;
  delay(500);
  gripperclose();
  fwrd();
  delay(4250);
  stp();
  right(85);
  fwrd();
  delay(2500);
  stp();
  //targetpos = 90;
  gripperclose();
  right(90);
  //fwrd();
  //delay(1200);
  gripperopen();
  delay(250);
  rev();
  delay(500);
  stp();
}