void xyAdiferencial()
{
  int baseSpeed;
  int turnDelta;

  // velocidad base
  baseSpeed = map ( pad_y, MIN_RAW_ADC, MAX_RAW_ADC, MAX_SPEED_SETTING, MIN_SPEED_SETTING );
  // valor diferencial para girar
  turnDelta = map ( pad_x, MIN_RAW_ADC, MAX_RAW_ADC, MIN_TURN_DELTA, MAX_TURN_DELTA );

  Ispeed = baseSpeed + turnDelta;
  Dspeed = baseSpeed - turnDelta;
  MoverOrugas ( Ispeed, Dspeed);
}
void MoverOrugas (int Ispeed, int Dspeed) {
  if (Ispeed < 0) {
    digitalWrite(IZQ_AVZ, LOW);
    digitalWrite(IZQ_RET, HIGH);
  } else {
    digitalWrite(IZQ_AVZ, HIGH);
    digitalWrite(IZQ_RET, LOW);
  }
  if (Dspeed < 0) {
    digitalWrite(DER_AVZ, LOW);
    digitalWrite(DER_RET, HIGH);
  } else {
    digitalWrite(DER_AVZ, HIGH);
    digitalWrite(DER_RET, LOW);
  }
  
  Ispeed = abs (Ispeed);
  Dspeed = abs (Dspeed);
  Ispeed = constrain (Ispeed, 0, 1022);
  Dspeed = constrain (Dspeed, 0, 1023);
  ledcWrite (IZQ_PWM_Ch, Ispeed);
  ledcWrite (DER_PWM_Ch, Dspeed);
}
