long t = 0;          // for keeping track of the time elapsed, milliseconds
long tint = 120000;  // step time interval milliseconds
long tlast = 0;
int PIN_PWM = 3;     // PWM output pin
int pwm_val = 0;
int pwm_max = 255;
int pwm_min = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  long a = millis() % tint;
  float pos = (float) a / tint;
  int val = (int) pwm_max * pos;
  if (val != pwm_val) {
    analogWrite(PIN_PWM,val);
    Serial.print(a);
    Serial.print(',');
    Serial.print(pos);
    Serial.print(',');
    Serial.println(val);
    
    pwm_val = val;
  }
  if (pwm_val >= pwm_max) {
    pwm_val = pwm_min;
  }
}
