/* Read Quadrature Encoder
* Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.
*
* Sketch by max wolf / www.meso.net
* v. 0.1 - very basic functions - mw 20061220
*
*/
#include "Timer.h"

int encoder0PinA = 2;
int encoder0PinB = 11;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
volatile int n = LOW;
int last = 0;

int encoder1PinA = 5;
int encoder1PinB = A2;
int encoder1Pos = 0;
int encoder1PinALast = LOW;
volatile int n2 = LOW;
int last1 = 0;

Timer t;

void doLeft()
{
    n = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (n == HIGH))
    {
        if (analogRead(encoder1PinB) < 450)
        {
            encoder1Pos--;
        }
        else
        {
            encoder1Pos++;
        }
    }
    encoder1PinALast = n;
}

void doRight()
{
    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH))
    {
        if (digitalRead(encoder0PinB) == LOW)
        {
            //if (analogRead(encoder0PinB) < 450) {
            encoder0Pos--;
        }
        else
        {
            encoder0Pos++;
        }
    }
    encoder0PinALast = n;
}

void printStuff()
{
    if (last != encoder0Pos)
    {
        Serial.print ("A");
        Serial.println (encoder0Pos);
//        Serial.print ("/");
        last = encoder0Pos;
    }

    if (last1 != encoder1Pos)
    {
        Serial.print ("B");
        Serial.println (encoder1Pos);
//        Serial.print ("/");
        last1 = encoder1Pos;
    }
}

void setup()
{
    pinMode (encoder0PinA, INPUT);
    pinMode (encoder0PinB, INPUT);
    pinMode (encoder1PinA, INPUT);
    // pinMode (encoder1PinB, INPUT); // Analog

    attachInterrupt(0, doRight, CHANGE);
    Serial.begin (9600);

    t.every(1000, printStuff);
}

void loop()
{
    doLeft();

    t.update();
}
