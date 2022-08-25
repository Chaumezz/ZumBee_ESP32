void alcanzarPosicion (int PosFinal) {

  if (PosFinal >= 0 && PosFinal < 180)
  {
    while (PosFinal - 5 >= int(orientacionSensorBno055()))
    {
      MoverOrugas ( 350, -350);
    }
  }
    if (PosFinal >= 180 && PosFinal <= 360)
    {
      while (PosFinal + 5 <= int(orientacionSensorBno055()))
      {
        MoverOrugas ( -350, 350);
      }
    }
    else{

    digitalWrite(IZQ_AVZ, LOW);
    digitalWrite(DER_AVZ, LOW);
    digitalWrite(IZQ_RET, LOW);
    digitalWrite(DER_RET, LOW);
  }
}


int orientacionSensorBno055() {
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return (orientationData.orientation.x);
}
