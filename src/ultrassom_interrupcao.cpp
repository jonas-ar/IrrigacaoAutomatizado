#include "ultrassom_interrupcao.hpp"

static volatile unsigned long tempoInicio = 0;
static volatile unsigned long duracao = 0;
static volatile bool pulsoFinalizado = false;
static uint8_t pinEcho;
static uint8_t pinTrig;

void IRAM_ATTR echoChange() {
  if (digitalRead(pinEcho) == HIGH) {
    tempoInicio = micros();
  } else {
    duracao = micros() - tempoInicio;
    pulsoFinalizado = true;
  }
}

void setupUltrassom(uint8_t trigPin, uint8_t echoPin) {
  pinTrig = trigPin;
  pinEcho = echoPin;

  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinEcho), echoChange, CHANGE);
}

float medirDistancia() {
  pulsoFinalizado = false;

  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  unsigned long tempoLimite = millis() + 50;
  while (!pulsoFinalizado && millis() < tempoLimite) {
    delay(1);
  }

  float distancia = (duracao * 0.0343) / 2.0;
  return pulsoFinalizado ? distancia : -1.0; // -1.0 indica falha
}