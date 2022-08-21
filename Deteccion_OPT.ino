void esquivaObstaculos ()
{
  if (sensor.isSampleDone())
  {
    sensor.readOutputRegs();
    distances[sensor.channelUsed] = sensor.distanceMillimeters;
    sensor.nextChannel();
    sensor.startSample();
  }

  if (distances[1]  >= 300 && distances[1]  < 400)
    MoverOrugas ( 1000, 1000);

  else if (distances[1]  >= 190 && distances[1]  < 300)
    MoverOrugas ( 900, 900);

  else if (distances[1]  >= 160 && distances[1]  < 190)
    MoverOrugas ( 760, 760);

  else if (distances[1]  >= 140 && distances[1]  < 160)
    MoverOrugas ( 600, 600);

  else if (distances[1]  < 140)
  {
    if (distances [0] < distances [2])
    {
      giraDch ();
      delay (80);
    }
    else
    {
      giraIzq ();
      delay (80);
    }
  }
  else
  {
    MoverOrugas ( 1021, 1023);
  }
}




/*



  void esquivaObstaculos ()
  {
  if (distances[0] && distances[1] && distances[2] > 300)
  {
   MoverOrugas ( 1023, 1023);
  }
  if (distances[0] && distances[1] && distances[2] < 300)
  {
  MoverOrugas ( 800, 800);
  }
  if (distances[0] && distances[1] && distances[2] < 200)
  {
   MoverOrugas ( 600, 600);
  }
  if (distances[1] < 120  )
  {
    if (distances [0] <= distances [2]) giraDch ();
    else giraIzq ();
  }
  if (distances[0] < 110 )
  {
  MoverOrugas ( 600, 0);
  }
  if ( distances[2] < 110 )
  {
  MoverOrugas ( 0, 600);
  }
  }


  if ( distances[1] < 250  && distances[1]  > 150 )
  {
    digitalWrite(DER_RET, LOW);       digitalWrite(IZQ_RET, LOW);
    ledcWrite (DER_AVZ_PWM_Ch, 900); ledcWrite (IZQ_AVZ_PWM_Ch, 900);
    Serial.println ("900");
  }


  if (distances[0] < 110 )
  {
  digitalWrite(DER_RET, LOW);       digitalWrite(IZQ_RET, LOW);
  ledcWrite (DER_AVZ_PWM_Ch, 0); ledcWrite (IZQ_AVZ_PWM_Ch, 800);
  }
  if ( distances[2] < 110 )
  {
  digitalWrite(DER_RET, LOW);       digitalWrite(IZQ_RET, LOW);
  ledcWrite (DER_AVZ_PWM_Ch, 800); ledcWrite (IZQ_AVZ_PWM_Ch, 0);
  }
*/



void giraDch () {
  SetpointL = counter_L + 650;
  SetpointR = counter_R - 650;
  myPID_L.Compute();
  myPID_R.Compute();
  MoverOrugas ( OutputL, OutputR);
}
void giraIzq () {
  SetpointL = counter_L - 650 ;
  SetpointR = counter_R + 650 ;
  myPID_L.Compute();
  myPID_R.Compute();
  MoverOrugas ( OutputL, OutputR);
}
void paroMotores () {
  MoverOrugas ( 0, 0);
}
