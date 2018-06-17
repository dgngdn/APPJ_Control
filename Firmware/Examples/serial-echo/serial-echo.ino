String mystring;


void setup() {
  Serial.begin(9600);
}

void loop() {
  bool stringcomplete = false;
  while (Serial.available() > 0) {
    char inchar = (char)Serial.read();
    mystring += inchar;
    if (inchar == '\n') {
      stringcomplete = true;
      break;
      }
    }
  if (stringcomplete == true) {
    Serial.print("re-echoing: ");
    Serial.print(mystring);
    mystring = "";
    
  }
  delay(1);
}
