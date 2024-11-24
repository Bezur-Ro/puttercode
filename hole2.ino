/*
		---uses gyro for turning
		---uses Line follower to track stright portion of course
*/


void hole2() {
  uint32_t period = 1000L;  // 1 second.
  POW = 67;
  delay(250);
  gripperclose();
  fwrd();
  delay(1000);
  stp();
  for (uint32_t tStart = millis(); (millis() - tStart) < period;) {  // track line for "interval" amount of time
    delay(10);
    Application_FunctionSet.ApplicationFunctionSet_Tracking();
    Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  }
  stp();
  fwrd();
  delay(1250);
  stp();
  //targetpos = 65;
  delay(200);
  right(65);
  delay(200);
  fwrd();
  delay(800);
  stp();
  delay(300);
  gripperopen();
  delay(300);
  rev();
  delay(1000);
  stp();
}