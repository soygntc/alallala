
const int stepPin = 5; 
const int dirPin  = 2;
const int enPin   = 7;

// Parámetros de tiempo
const unsigned long duracionMaceracion = 60UL * 60UL * 1000UL;  // 60 minutos en milisegundos (UL: Unsigned Long como enteros largos sin signo)

// Velocidad del motor
const int microsegundosPaso =  37;  // ajusta para cambiar RPM

unsigned long tiempoInicio;

void setup() {
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT); 
  pinMode(enPin, OUTPUT); 

  // Dirección y habilitación del driver
  digitalWrite(dirPin, HIGH); // direccion del eje (clockwise)
  digitalWrite(enPin, LOW); // Habilita el driver (motor encendido)

  tiempoInicio = millis(); // Guarda el tiempo de inicio
}

void loop() {
  // Verifica si ha pasado el tiempo de maceración
  if (millis() - tiempoInicio < duracionMaceracion) {
    // Motor activo: gira continuamente
    digitalWrite(stepPin, HIGH); // Marca el inicio del pulso 
    delayMicroseconds(microsegundosPaso);
    digitalWrite(stepPin, LOW); // Marca el final  del pulso 
    delayMicroseconds(microsegundosPaso);
  } else {
    // Se terminó el proceso: se desactiva el driver
    digitalWrite(enPin, HIGH); // Deshabilita el driver (motor se apaga)
    while (true); // Detiene el programa para siempre
  }
}
