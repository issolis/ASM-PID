

// Pines del sensor ultrasónico
const int trigPin = 12;
const int echoPin = 11;

// Pines para las bombas
const int bombaEntradaPin = 9;  // Para llenar el agua
const int bombaSalidaPin = 10;  // Para vaciar el agua

// Constantes del controlador PID
double Kp = 1.0;  // Ganancia proporcional
double Ki = 0.1;  // Ganancia integral
double Kd = 0.05; // Ganancia derivativa

// Variables del sistema PID
double setpoint = 6.0;   // Nivel objetivo en centímetros
double input;             // Nivel medido (entrada para el PID)
double output;            // Salida del controlador PID
double lastError = 0.0;   // Error anterior para el término derivativo
double integral = 0.0;    // Acumulador del término integral

// Variables de tiempo
unsigned long lastTime = 0;
unsigned long sampleTime = 10; // Tiempo de muestreo en milisegundos

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Configurar los pines de las bombas como salidas
  pinMode(bombaEntradaPin, OUTPUT);
  pinMode(bombaSalidaPin, OUTPUT);
  
  // Inicialmente, apagamos las bombas
  analogWrite(bombaEntradaPin, 0);
  analogWrite(bombaSalidaPin, 0);
  
  Serial.begin(9600);
}

void loop() {
  // Leer el nivel del agua con el sensor ultrasónico
  input = medirNivelAgua(); 
  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;

  setpoint = (analogRead(A0) + 1)/128 + 4;
  


  if (timeChange >= sampleTime) {
    double error = setpoint - input;

    integral += error * (timeChange / 1000.0); 
    double derivative = (error - lastError) / (timeChange / 1000.0);
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (error < 0.30 && error >-0.30){
       analogWrite(bombaSalidaPin, 0);
       analogWrite(bombaEntradaPin, 0); 
    }else{
      if (error > 0) {
        analogWrite(bombaEntradaPin, constrain(output, 128, 255));
        analogWrite(bombaSalidaPin, 0); // Apagar la bomba de salida
      } else {
        analogWrite(bombaSalidaPin, constrain(-output, 128, 255));
        analogWrite(bombaEntradaPin, 0); 
      }
    }

    lastError = error;
    lastTime = now;

    Serial.print("Nivel del Agua (cm): "); Serial.print(input);
    Serial.print(" | error: "); Serial.print(error);
    Serial.print(" | Setpoint: "); Serial.print(setpoint);
    Serial.print(" | Salida PID: "); Serial.println(output);
  }
}

// Función para medir el nivel de agua usando el sensor ultrasónico
double medirNivelAgua() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Leer el tiempo de ida y vuelta de la señal
  long duration = pulseIn(echoPin, HIGH);

  // Calcular la distancia en centímetros
  double distance = (duration * 0.034) / 2;

  return distance; // Nivel del agua en cm
}

double getSetPoint(){
  
  double selector = (analogRead(A2) + 1)/204.8; 

  if (selector == 0)
    return (analogRead(A0) + 1)/128 + 4;
  else if (selector == 1)
    return 



}
