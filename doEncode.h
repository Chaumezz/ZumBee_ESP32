void doEncodeA()
{
  if (millis() > timeCounter + timeThreshold)
  {
    if (digitalRead(IntPin_A) == digitalRead(IntPin_B))
    {
      IsCW_L = true;
      ISRCounter_L++;
    }
    else
    {
      IsCW_L = false;
       ISRCounter_L--;
    }
    timeCounter = millis();
  }
}

void doEncodeB()
{
  if (millis() > timeCounter + timeThreshold)
  {
    if (digitalRead(IntPin_A) != digitalRead(IntPin_B))
    {
      IsCW_L = true;
     ISRCounter_L++;
    }
    else
    {
      IsCW_L = false;
     ISRCounter_L--;
    }
    timeCounter = millis();
  }
}

void doEncodeC()
{
  if (millis() > timeCounter + timeThreshold)
  {
    if (digitalRead(IntPin_C) == digitalRead(IntPin_D))
    {
      IsCW_R = true;
       ISRCounter_R++;
    }
    else
    {
      IsCW_R = false;
      ISRCounter_R--;
    }
    timeCounter = millis();
  }
}

void doEncodeD()
{
  if (millis() > timeCounter + timeThreshold)
  {
    if (digitalRead(IntPin_C) != digitalRead(IntPin_D))
    {
      IsCW_L = true;
       ISRCounter_R++;
    }
    else
    {
      IsCW_R = false;
     ISRCounter_R--;
    }
    timeCounter = millis();
  }
}
