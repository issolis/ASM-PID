

// Pines del sensor ultrasónico
const int trigPin = 12;
const int echoPin = 11;

// Pines para las bombas
const int inputBombPin = 9;  
const int outputBombPin = 10;  

// Constantes del controlador PID

int amp = 1;
double Kp = 5;  
double Ki = 1;  
double Kd = 0.05; 

// Variables del sistema PID
double setpoint ;   
double input;             
double output;            
double lastError = 0.0;   
double integral = 0.0;    

// Variables de tiempo
unsigned long lastTime = 0;
unsigned long sampleTime = 10; 

unsigned long startTime;
const unsigned long cycleDuration = 60000;  //20s

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(inputBombPin, OUTPUT);
  pinMode(outputBombPin, OUTPUT);
  
  analogWrite(inputBombPin, 0);
  analogWrite(outputBombPin, 0);
  
  Serial.begin(9600);
}

void loop() {
  input = measureWaterLevel(); 
  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;

  unsigned long elapsedTime = now - startTime;


  if (elapsedTime >= cycleDuration) {
    startTime = now;  
    elapsedTime = 0;
  }

  float cycleProgress = (float)elapsedTime / cycleDuration;
  int option = analogRead(A2) / 250;
  setpoint =  getSetpoint(option, cycleProgress);

  


  if (timeChange >= sampleTime) {
    double error = setpoint - input;

    integral += error * (timeChange / 1000.0); 
    double derivative = (error - lastError) / (timeChange / 1000.0);
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (error < 0.20 && error >-0.20){
       analogWrite(outputBombPin, 0);
       analogWrite(inputBombPin, 0); 
    }else{
      if (error > 0) {
        analogWrite(inputBombPin, 255);
        analogWrite(outputBombPin, 0); // Apagar la bomba de salida
      } else {
        analogWrite(outputBombPin, 255);
        analogWrite(inputBombPin, 0); 
      }
    }

    lastError = error;
    lastTime = now;

    Serial.print("Water level (cm): "); Serial.print(input);
    Serial.print(" | Option: "); Serial.print(option); 
    Serial.print(" | error: "); Serial.print(error);
    Serial.print(" | Setpoint: "); Serial.print(setpoint);
    Serial.print(" | PID output: "); Serial.println(output);
  }
}

// Función para medir el nivel de agua usando el sensor ultrasónico
double measureWaterLevel() {

  digitalWrite(trigPin, LOW);
  
  unsigned long now = micros();
  while(micros()-now <= 2);
  digitalWrite(trigPin, HIGH);

  now = micros();
  while(micros()-now <= 10);
  digitalWrite(trigPin, LOW);
  
  // Leer el tiempo de ida y vuelta de la señal
  long duration = pulseIn(echoPin, HIGH);

  // Calcular la distancia en centímetros
  double distance = (duration * 0.034) / 2;

  return distance; // Nivel del agua en cm
}

float getSetpoint(int option, float progress){
  if (option == 0)
    return ((analogRead(A0) + 1)/(1000/6) + 4);
  else if (option == 1)
    return generateSquare(progress); 
  else if (option == 2)
    return generateTriangular(progress); 
  else if (option == 3)
    return generateSine(progress); 
  else 
    return generateSawtoothValue(progress); 
}


int generateSquare(float progress) {
  int squareState = (progress < 0.5) ?  10 : 5 ;  
  //Serial.println(squareState);
  return squareState; 
}

int generateTriangular(float progress) {
  float triangleValue = (progress < 0.5) ? (progress * 2 * 255) : ((1 - progress) * 2 * 255);
  triangleValue = (int) (triangleValue) / 64 + 5;  
  //Serial.println(triangleValue);
  return triangleValue; 
}

int generateSine(float progress) {
  float sineValue = 127.5 * (1 + sin(2 * PI * progress));  
  sineValue =(int)(sineValue/42.5 + 4); 
  //Serial.println(sineValue);
  return sineValue;
}

int generateSawtoothValue(float progress) {
  float sawtoothValue = progress * 255;  
  sawtoothValue = (int) (sawtoothValue/(255/5) + 5);
  //Serial.println(sawtoothValue);
  return sawtoothValue; 
}
