// **********************************************  ZUMBEE_ESP32 SKETCH PARA AJUSTAR PID EN MOTORES N20   **********************************************************************
// **********************************************       en Agosto 2022         **********************************************************************

#include <PID_v1.h>                  // Librería PID de Brett Beauregard: https://playground.arduino.cc/Code/PIDLibrary

// **********************************************  Patillaje ESP32 DevKit   **********************************************************************
#define IntPin_A   39
#define IntPin_B   36

#define IZQ_AVZ      33
#define IZQ_RET      32
#define IZQ_PWM      27

void ICACHE_RAM_ATTR doEncodeA();
void ICACHE_RAM_ATTR doEncodeB();
bool IsCW_L = true;

const int IZQ_PWM_Ch = 2;
const int PWM_Res = 10;
const int PWM_Freq = 20000;
// ************************************************ Variables PID *****************************************************************
double        Setpoint = 0.0, Input = 0.0, Output = 0.0;  // Setpoint=Posición designada; Input=Posición del motor; Output=Tensión de salida para el motor.
double        kp = 0.0, ki = 0.0, kd = 0.0;               // Constante proporcional, integral y derivativa.
double        outMax = 0.0, outMin = 0.0;                 // Límites para no sobrepasar la resolución del PWM.
// **************************************************** Otras Variables ***********************************************************
volatile long contador = 0;           // En esta variable se guardará los pulsos del encoder y que interpreremos como distancia (o ángulo si ese fuese el caso).
byte          ant = 0, act = 0;       // Sólo se utiliza los dos primeros bits de estas variables y servirán para decodificar el encoder. (ant=anterior, act=actual.)
byte          cmd = 0;                // Un byte que utilizamos para la comunicación serie. (cmd=comando.)
unsigned int  tmp = 0;                // Variable que utilizaremos para poner el tiempo de muestreo.
//  const byte    ledok = 13;             // El pin 13 de los Arduinos tienen un led que utilizo para mostrar que el motor ya ha llegado a la posición designada.
// ********************************************************************************************************************************

PID myPID(&Input, &Output, &Setpoint, 0.0, 0.0, 0.0,P_ON_M, DIRECT); // Parámetros y configuración para invocar la librería.

void setup()                          // Inicializamos todo las variables que sean necesarias, configuramos los pines de entrada/salida y configuramos el terminal serie.
{
  pinMode(IntPin_A, INPUT_PULLUP);
  pinMode(IntPin_B, INPUT_PULLUP);
  pinMode (IZQ_PWM, OUTPUT);
  pinMode (IZQ_AVZ, OUTPUT);
  pinMode (IZQ_RET, OUTPUT);

  attachInterrupt(IntPin_A, doEncodeA, CHANGE);
  attachInterrupt(IntPin_B, doEncodeB, CHANGE);

  ledcSetup(IZQ_PWM_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(IZQ_PWM, IZQ_PWM_Ch);
  digitalWrite(IZQ_AVZ, LOW);
  digitalWrite(IZQ_RET, LOW);


  Serial.begin(115200);               // Configura la velocidad en baudios del terminal serie. Hoy en día "115200" es soportada por la gran mayoría de computadores.

  outMax =  1023.0;                    // Límite máximo del controlador PID.
  outMin = -outMax;                   // Límite mínimo del controlador PID.

  tmp = 10;                           // Tiempo de muestreo en milisegundos.

  kp = 4.0;                          // Constantes PID iniciales.
  ki = 6.0;                           // el lector de encoder está diseñado como x4 pq lee CHANGE
 kd = 0.4
  myPID.SetSampleTime(tmp);           // Envía a la librería el tiempo de muestreo.
  myPID.SetOutputLimits(outMin, outMax);// Límites máximo y mínimo; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). Ambos PWM se convertirán a la salida en valores absolutos, nunca negativos.
  myPID.SetTunings(kp, ki, kd);       // Constantes de sintonización.
  myPID.SetMode(AUTOMATIC);           // Habilita el control PID (por defecto).
  Setpoint = (double)contador;        // Para evitar que haga cosas extrañas al inciarse, igualamos los dos valores para que comience estando el motor parado.

  imprimir(3);                        // Muestra los datos de sintonización y el tiempo de muestreo por el terminal serie.
}

void loop()
{
  Input = (double)contador;           // Lectura del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones extrenas (pines 2 y 3).

  while (!myPID.Compute());           // Mientras no se cumpla el tiempo de muestreo, se queda en este bucle.

  // *********************************************** Control del Motor *************************************************
  if (((long)Setpoint - contador) == 0)// Cuando está en el punto designado, parar el motor.
  {
    MoverOrugas(0);
  }
  else                                // En caso contrario hemos de ver si el motor ha de ir hacia delante o hacia atrás. Esto lo determina el signo de la variable "Output".
  {
    MoverOrugas(Output);
  }

  // Recepción de datos para posicionar el motor, o modificar las constantes PID, o el tiempo de muestreo. Admite posiciones relativas y absolutas.
  if (Serial.available() > 0)           // Comprueba si ha recibido algún dato por el terminal serie.
  {
    cmd = 0;                            // Por seguridad "limpiamos" cmd.
    cmd = Serial.read();                // "cmd" guarda el byte recibido.
    if (cmd > 31)
    {
      byte flags = 0;                                     // Borramos la bandera que decide lo que hay que imprimir.
      if (cmd >  'Z') cmd -= 32;                          // Si una letra entra en minúscula la covierte en mayúscula.
      if (cmd == 'W') {
        Setpoint += 100.0;      // Si (por ejemplo) es la letra 'W' mueve 100 pasos hacia delante. Estos son movimientos relativos.
        flags = 2;
      }
      if (cmd == 'Q') {
        Setpoint -= 100.0;      // Aquí son esos 100 pasos pero hacia atrás si se pulsa la letra 'Q'.
        flags = 2;
      }
      if (cmd == 'S') {
        Setpoint += 1000.0;    // Se repite lo mismo en el resto de las teclas.
        flags = 2;
      }
      if (cmd == 'A') {
        Setpoint -= 1000.0;
        flags = 2;
      }
      if (cmd == 'X') {
        Setpoint += 5000.0;
        flags = 2;
      }
      if (cmd == 'Z') {
        Setpoint -= 5000.0;
        flags = 2;
      }
      if (cmd == '2') {
        Setpoint += 15000.0;
        flags = 2;
      }
      if (cmd == '1') {
        Setpoint -= 15000.0;
        flags = 2;
      }

      // Decodificador para modificar las constantes PID.
      switch (cmd)                                                                           // Si ponemos en el terminal serie, por ejemplo "p2.5 i0.5 d40" y pulsas enter  tomará esos valores y los cargará en kp, ki y kd.
      { // También se puede poner individualmente, por ejemplo "p5.5", sólo cambiará el parámetro kp, los mismo si son de dos en dos.
        case 'P': kp  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break; // Carga las constantes y presenta en el terminal serie los valores de las variables que hayan sido modificadas.
        case 'I': ki  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break;
        case 'D': kd  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); flags = 1; break;
        case 'T': tmp = Serial.parseInt();   myPID.SetSampleTime(tmp);     flags = 1; break;
        case 'G': Setpoint = Serial.parseFloat();                          flags = 2; break;  // Esta línea permite introducir una posición absoluta. Ex: g13360 (y luego enter) e irá a esa posición.
        case 'K':                                                          flags = 3; break;
      }
      //    digitalWrite(ledok, LOW);       // Cuando entra una posición nueva se apaga el led y no se volverá a encender hasta que el motor llegue a la posición que le hayamos designado.

      imprimir(flags);
    }
  }
}


void imprimir(byte flag) // Imprime en el terminal serie los datos de las contantes PID, tiempo de muestreo y posición. En los demás casos sólo imprime la posición del motor.
{
  if ((flag == 1) || (flag == 3))
  {
    Serial.print("KP=");     Serial.print(kp);
    Serial.print(" KI=");    Serial.print(ki);
    Serial.print(" KD=");    Serial.print(kd);
    Serial.print(" Time=");  Serial.println(tmp);
  }
  if ((flag == 2) || (flag == 3))
  {
    Serial.print("Posicion:");
    Serial.println((long)Setpoint);
  }
}

void doEncodeA()
{
    if (digitalRead(IntPin_A) == digitalRead(IntPin_B))
    {
      IsCW_L = true;
      contador++;
    }
    else
    {
      IsCW_L = false;
       contador--;
    }
}

void doEncodeB()
{
    if (digitalRead(IntPin_A) != digitalRead(IntPin_B))
    {
      IsCW_L = true;
     contador++;
    }
    else
    {
      IsCW_L = false;
     contador--;
    }
}


void MoverOrugas (int Ispeed) {
  if (Ispeed < 0) {
    Ispeed = abs (Ispeed);
    digitalWrite(IZQ_AVZ, LOW);
    digitalWrite(IZQ_RET, HIGH);
  } else {
    digitalWrite(IZQ_AVZ, HIGH);
    digitalWrite(IZQ_RET, LOW);
  }
  Ispeed = constrain (Ispeed, 0, 1023);
  ledcWrite (IZQ_PWM_Ch, Ispeed);
}
