long t = 0;          // for keeping track of the time elapsed, milliseconds
long tint = 300000;  // step time interval milliseconds
long tlast = 0;
int PIN_PWM = 3;     // PWM output pin
int PWM_VALS[] = {255-255, 255-128, 255-64, 255-160, 255-92, 255-255};
int pwm_position = 0;

void setup() {
  // put your setup code here, to run once:
  analogWrite(PIN_PWM,PWM_VALS[pwm_position]);
}

void loop() {
  // put your main code here, to run repeatedly:
  t = millis();
  if (t-tlast >= tint) {
    tlast = t;
    pwm_position++;
    if (pwm_position >= sizeof(PWM_VALS)/sizeof(PWM_VALS[0])) {
      pwm_position = 0;
    }
    analogWrite(PIN_PWM,PWM_VALS[pwm_position]);
  }
}
