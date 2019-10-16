#include <EEPROM.h>

String temp1 = "bys+12345678+id01";
int alldata;
char charbuf[512];
String jadi;

void setup() {
  // put your setup code here, to run once:
  EEPROM.begin(512);
  alldata = temp1.length();
  Serial.println("alldata : " + alldata);
  delay(10);
  temp1.toCharArray(charbuf, alldata + 1);
  writeflash();
  Serial.begin(9600);
}

void loop() {
  readflash();
  Serial.println("jadi seperti ini : " + jadi);
  delay(1000);
}

void writeflash() {
  // write a 0 to all 512 bytes of the EEPROM
  for (int i = 0; i < alldata; i++) {
    EEPROM.write(i, charbuf[i]);
    Serial.println(charbuf[i]);
  }
}

void readflash() {
  jadi = "";
  for (int i = 0; i < alldata; i++) {
    int value = EEPROM.read(i);
//    value1 = String(value, char);
    jadi += (char)value;
    Serial.print(i);
    Serial.print("\t");
    Serial.print((char)value);
    Serial.println();
    // there are only 512 bytes of EEPROM, from 0 to 511, so if we're
    // on address 512, wrap around to address 0
    if (i == 512) {
      i = 0;
      delay(10);
    }
  }
}
