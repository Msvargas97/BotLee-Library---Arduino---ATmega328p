
#include <BotLee.h>

/* S0    S1      S2      S3      S4... */ //DISABLE  DIGITAL


BotLeeSensors sensors; //Crea un objeto para utilizar los 8 sensores
BotLeeSerial serial;
BotLeeOLED oled;

void setup() {
  // put your setup code here, to run once:
  sensors.init();
  oled.begin();
  serial.begin(9600);
  serial.println("BOT-LEE START");
  oled.initBuffer();
  oled.clearDisplay();
}
int val[NUM_SENSORS];
void loop() {
  // put your main code here, to run repeatedly
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    val[i] = !sensors.readSensor(i);
    serial.print("%d ", val[i]);
  }
  serial.println();
  drawSensors(val);
  //delay(1000);

}

void drawSensors(int nPer[NUM_SENSORS]) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE, BLACK);
  char offset = 0;
  char j;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (i >= 4) {
      j = i - 4;
      offset =  oled.height() / 2 + 2;
      j = i - 4;
      oled.setCursor(7 + j * 29, offset); //cursor x,y
    } else {
      oled.setCursor(7 + i * 29, offset); //cursor x,y
    }
    oled.print("S");
    oled.print(i);
    if (i < 4) {
      if (nPer[i] >= 1) oled.fillRect(2 + i * 28, 8, 24 , oled.height() / 2 - 8, WHITE);
      else oled.fillRect(2 + i * 28, 8, 24 , oled.height() / 2 - 8, BLACK);
      oled.drawFastVLine(28 + i * 28, 0, oled.height(), WHITE);
    } else {
      j = i - 4;
      if (nPer[i] >= 1) oled.fillRect(2 + j * 28, 8 + offset, 24 , oled.height()-1, WHITE);
      else oled.fillRect(2 + j * 28, 8 + offset, 24 , oled.height()-1, BLACK);
    }
  }
  oled.drawFastHLine(0, oled.height() / 2 + 1, oled.width(), WHITE);
  oled.show();
}
