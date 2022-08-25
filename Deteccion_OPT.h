void esquivaObstaculos ()
{
   if (dataReady)
  {
    sensor.readOutputRegs();
    distances[sensor.channelUsed] = sensor.distanceMillimeters;
    dataReady = false;
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
   if (distances [0] < 120 && distances [1] < 120 && distances [2] < 120)
  {
    retrocede();
    delay (100);
  }
}

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

void retrocede () {
  MoverOrugas ( -800, -800);
}

void paroMotores () {
  MoverOrugas ( 0, 0);
}
