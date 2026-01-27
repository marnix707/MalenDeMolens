```/* ================================================================
   GEMAALROTATIE SENSOR (ATtiny I2C Slave)           -        V1.0
   AUTHOR:                                          MARNIX HARMSEN
   ================================================================ */
#include <Wire.h>

// Pinout
#define pinBJT    2
#define inputIR  3
#define pinBlauw 10

#define I2C_ADDRESS 0x36   // Decimaal 54

// --- VARIABELEN ---
volatile uint32_t gemaalrotatie = 0;
volatile uint8_t  lastCommand   = 0;
volatile uint32_t startTime     = 0;
volatile uint32_t lastPulseTime = 0;
volatile uint8_t aantalMillisFilter = 500;
volatile uint32_t ledOffTime = 0;


boolean isOnderbroken = false;

void setup() {
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  pinMode(pinBJT, OUTPUT);
  pinMode(pinBlauw, OUTPUT);
  pinMode(inputIR, INPUT);

  digitalWrite(pinBJT, HIGH);
  startTime = millis();
}

void loop() {
  if (digitalRead(inputIR) == LOW) {
    digitalWrite(pinBlauw, LOW);
    if (!isOnderbroken) {
      gemaalrotatie++;
      isOnderbroken = true;
    }
  } else {
    digitalWrite(pinBlauw, HIGH);
    isOnderbroken = false;
  }
}



void receiveEvent() {
  if (Wire.available()) {
    lastCommand = Wire.read();
    while (Wire.available()) Wire.read();
  }
}

void requestEvent() {
  if (lastCommand == 0x11) {

    // Maak lokale kopieën (ISR-veilig)
    uint32_t countCopy = gemaalrotatie;
    uint32_t timeNow   = millis();
    uint32_t deltaTime = timeNow - startTime;

    uint8_t buffer[9];
    buffer[0] = 8; // 8 databijtes volgen

    // Elapsed time (ms)
    buffer[1] = (deltaTime >> 24) & 0xFF;
    buffer[2] = (deltaTime >> 16) & 0xFF;
    buffer[3] = (deltaTime >> 8)  & 0xFF;
    buffer[4] =  deltaTime        & 0xFF;

    // Gemaalrotatie count
    buffer[5] = (countCopy >> 24) & 0xFF;
    buffer[6] = (countCopy >> 16) & 0xFF;
    buffer[7] = (countCopy >> 8)  & 0xFF;
    buffer[8] =  countCopy        & 0xFF;

    Wire.write(buffer, 9);

    // Reset NA succesvol verzenden
    gemaalrotatie = 0;
    startTime     = timeNow;
  }
  else {
    uint8_t err = 0;
    Wire.write(&err, 1);
  }

  lastCommand = 0;
}```
