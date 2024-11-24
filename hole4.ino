/*
		---uses gyro for turning
*/


void hole4() {

  POW = 50;
  tpow = 50;
  //targetpos = 15;
  delay(500);
  gripperclose();
  fwrd();
  delay(1500);
  stp();
  right(15);
  right(15);
  delay(500);
  fwrd();
  delay(650);
  stp();
  left(15);
  left(15);
  left(15);
  left(15);
  fwrd();
  delay(1100);
  stp();
  right(15);
  right(15);
  right(15);
  delay(250);
  left(15);
  delay(500);
  fwrd();
  delay(1400);
  stp();
  delay(400);
  gripperopen();
  delay(400);

  rev();
  delay(500);
  stp();
}
