void leerBT() {
  if (SerialBT.available()) {
    BluetoothData = SerialBT.read();
    if (BluetoothData == 'J') {
      mode = 1;
    }
    if (BluetoothData == 'D') {
      mode = 2;
    }
    if (BluetoothData == 'B') {
      mode = 3;
    }
    if (BluetoothData == 'H') {
      if (SerialBT.available()) {
        BluetoothData = SerialBT.read();
        if (BluetoothData == 'X') {
          pad_x = SerialBT.parseInt();
          while (BluetoothData != '*') {
            if (SerialBT.available()) {
              BluetoothData = SerialBT.read(); //Get next character from bluetooth
              if (BluetoothData == 'Y') pad_y = SerialBT.parseInt();
            }
          }
        }
      }
    }
  }
}
