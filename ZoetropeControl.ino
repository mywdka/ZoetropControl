/**
 * @file: ZoetropeControl.ino
 * @title: Zoetrope STROBE and Motor control
 * @author: Simon de Bakker <simon@simbits.nl>
 *
 * This sketch is for the Zoetrope module given by Oyo at the WdKA
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

/* The potmeter input pin */
#define POTMETER_IN  A0

/* The LED drive pin */
#define LED_STROBE_PIN  6 /* PD6 */

/* STROBE ON time */
#define LED_STROBE_ON_TIME  200  /* us */

/* Minimum and maximum strobe frequencies in Hz */
#define LED_STROBE_FREQ_MIN  0.5f
#define LED_STROBE_FREQ_MAX  4000.0f

/* Stepper RPM */
#define STEPPER_RPM  80

/* Stepper Steps per revolution */
#define STEPPER_SPR  200

/* Stepper port */
#define STEPPER_PORT 2

/* Stepper step type */
#define STEPPER_STEP_TYPE  DOUBLE

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *stepper;

int prevSensorReading;

/* Convert a frequency value to the TOP CTC value.
 * The effective range is 15625Hz to 0.23842Hz
 */
uint16_t freq_to_TOP(float hertz)
{
  uint16_t top;

  hertz = constrain(hertz, 0.23842f, 15625.0f);

  top = (F_CPU / (1024 * hertz) - 1);
  return top;
}

/* Sets the strobe frequency by changing the TOP value for the
 * Timer 1 compare match interrupt
 */
void set_strobe_freq(float hertz)
{
  uint16_t top = freq_to_TOP(hertz);
  OCR1A = top;
}

/*
 * This is the interrupt service routine for the Timer 1A compare match event
 * Here we will strobe the light for LED_STROBE_ON_TIME microseconds.
 * Remember to keep the interrupt routine as short as possible!
 */
ISR(TIMER1_COMPA_vect)
{
  PORTD |= (1 << LED_STROBE_PIN); /* semantically equivalent digitalWrite(LED_STROBE_PIN, HIGH), just faster */
  delayMicroseconds(LED_STROBE_ON_TIME);
  PORTD &= ~(1 << LED_STROBE_PIN); /* semantically equivalent digitalWrite(LED_STROBE_PIN, LOW), just faster */
}

void setup()
{
  /* set pin 6 of PORT D (Arduino pin D6) as OUTPUT */
  DDRD |= (1 << LED_STROBE_PIN); /* this is equivalent to saying: pinMode(LED_STROBE_PIN, OUTPUT); */

  /*
     F = F_CPU / (2 * N * (1 + OCRnA)) or
     With Prescaler set to 1024:
     Max F = 7812.5 KHz (OCRnA == 0)
     Min F = 0.119 Hz (TOP == 0xFFFF)

     CTC, OCR1A as TOP:
     WGM13: 0
     WGM12: 1
     WGM11: 0
     WGM10: 0

     Normal port operations:
     COM1A1: 0
     COM1A0: 0
     COM1B1: 0
     COM1B0: 0

     Prescaler 1024
     CS12: 1
     CS11: 0
     CS10: 1
  */

  /* We don't want interrupts to occur while setting up the timer */
  cli();

  /* Normal port operation */
  TCCR1A = 0;

  /* Set prescaler to 1024 */
  TCCR1B = (1<<CS12) | (1<<CS10);

  /* Set CTC (Clear Timer on Compare match) mode, OCR1A as TOP */
  TCCR1B |= (1<<WGM12);

  /* Start with a 'sane' frequency setting */
  OCR1A = freq_to_TOP(1);

  /* Enable the Timer 1A compare match interrupt
   * Every time the counter equals the TOP value (in the OCR1A register)
   * an interrupt (TIMER1_COMPA_vect) is issued.
   */
  TIMSK1 = (1<<OCIE1A);

  /* enable interrupts again */
  sei();

  /* stepper stuff */
  stepper = AFMS.getStepper(STEPPER_SPR, STEPPER_PORT);
  AFMS.begin();
  stepper->setSpeed(STEPPER_RPM);
}

void loop() {
  int sensorReading = analogRead(POTMETER_IN);

  /* FIXME: possibly some smoothing or hystheresis here? */
  if (sensorReading != prevSensorReading) {
    float freq = map(sensorReading, 0, 1023, LED_STROBE_FREQ_MIN, LED_STROBE_FREQ_MAX);
    set_strobe_freq(freq);
    prevSensorReading = sensorReading;
  }

  /* step step step */
  stepper->step(1, FORWARD, STEPPER_STEP_TYPE);
}
