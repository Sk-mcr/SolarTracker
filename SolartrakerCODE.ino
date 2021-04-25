
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MsTimer2.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Adafruit_ADS1015.h>
#include <EEPROM.h>

Adafruit_ADS1115 ads1115(0x49);
LiquidCrystal_I2C lcd(0x27, 16, 2);

Servo myservo;

//Automatik VAR
int Step_Auto = 0;
bool OutputSerial = 1;
long AutoTimer = 0;
long AutoTimeout = 3000;

int value = 0;
int temphandvalue = 0;
int handvalue = 0;
int Menuenummer = 0;
int tempMenuenummer = 0;
int hilfsvariable = 0;
int hilfsvariable1 = 0;
bool menue = false;

float value1 = 0.000;
float input_v = 0.00;
float input_i= 0.00;
float input_p= 0.00;
float input_w= 0.00;
float input_wh= 0.00;
float output_v= 0.00;
float output_i= 0.00;
float output_p= 0.00;
float output_w= 0.00;
float output_wh= 0.00;
float wirkungsgrad= 0.00;



// Joystick-PIN- Nummern
const int SW_pin = 7;
const int X_pin = A1;
const int Y_pin = A0;

int pos;
int actpos;
int xDir = 512;
int yDir = 512;



const int PIN_BLINK = 13;

//Steuerung Schwenkachse
const int PIN_Linksauf = 11;
const int PIN_Rechtslauf = 5;
const int PIN_EnableMotor = 12;




//Encoder Schwenkachse (DC_Motor)
const int PinCLK = 3;   // Erzeugen von unterbrechungen mit CLK-Signal
const int PinDT = 2;    // DT-Signal lesen
const int PinSW = 4;    // Signal Push Button Schalter

volatile boolean TurnDetected;  // Muss für unterbrechungen flüchtig sein
volatile boolean rotationdirection;  // CW- oder CCW-Drehung

int RotaryPosition = 0;  // Um die Position des Schrittmotors zu speichern

int PrevPosition;     // Vorherige Drehposition Wert zur Prüfung der Genauigkeit
int StepsToTake;      // Wie viel der Stepper sich bewegen muss


int obenlinks = 0; // Orientierung aus der Sicht von vorne
int obenrechts = 0;
int untenlinks = 0;
int untenrechts = 0;
int horizontalachse = 0;
int vertikalachse = 0;
int toleranzwert = 1700; //damit nicht ständig positioniert wird 800

long myTimer = 0;
long myTimeout = 1000;

// Interrupt-Routine läuft, wenn CLK von HIGH zu LOW geht
void isr ()  {
  delay(4);  // Verzögerung für Entprellung
  //  if (digitalRead(PinCLK))
  //    rotationdirection = digitalRead(PinDT);
  //  else
  //    rotationdirection = !digitalRead(PinDT);
  TurnDetected = true;
}





//Interrupt wird 1 mal pro sekunde ausgeführt
void timerInterrupt()
{
  //Messungen

  //Input von Solarpanel
  value1 = ((((analogRead(6) / 1024.0) * 5000) - 2500) / 185);
  
  input_i =abs(value1);

  input_v =25.0000 / 1023.0000 * analogRead(7);   //25V modul

  input_p = input_v * input_i;
  input_w = input_w + input_p;  ////Wattsekunde
  input_wh = input_w/ 3600 ;


  //Output von Laderegler(Verbrauch)
  value1 = ((((analogRead(3) / 1024.0) * 5000) - 2500) / 100);
  output_i = abs(value1);
  
  output_v =25.0000 / 1023.0000 * analogRead(2);   //25V modul

  output_p = output_v * output_i;
  output_w = output_w + output_p;  ////Wattsekunde
  output_wh = output_w/ 3600;

  wirkungsgrad = output_wh / input_wh;                //W_ab durch W_zu

  if (OutputSerial) {


    Serial.println("IN--------------------------------------------------------------------");

    Serial.println(input_i);
    Serial.println(input_v);
    Serial.println("Out------------------------------------------------------------");

    Serial.println(output_i);
    Serial.println(output_v);
  }
}


bool SteigendeFlanke(bool _Signal)
{
  static bool Flankenmerker;
  static bool Hilfsmerker;
  Flankenmerker = _Signal && !Hilfsmerker;
  Hilfsmerker   = _Signal;
  return Flankenmerker;
}

////////////////////////////////////////////////////////
//  ___      _
// / __| ___| |_ _  _ _ __
// \__ \/ -_)  _| || | '_ \
// |___/\___|\__|\_,_| .__/
//                   |_|
////////////////////////////////////////////////////////////////

void setup()
{
  // Robojax code for LCD with I2C
  // initialize the LCD,
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  // Robojax code for LCD with I2C
  lcd.clear();


  pinMode(7, INPUT_PULLUP);

  //MsTimer2::set(1000, timerInterrupt); // set the timer interrupt period 1000 ms
  //   MsTimer2::start(); // enable timer interrupt

  // put your setup code here, to run once:

  pinMode(PIN_Linksauf, OUTPUT);
  pinMode(PIN_Rechtslauf, OUTPUT);
  pinMode(PIN_EnableMotor, OUTPUT);
  pinMode(PIN_BLINK, OUTPUT);


  pinMode(PinCLK, INPUT);
  pinMode(PinDT, INPUT);
  pinMode(PinSW, INPUT);
  digitalWrite(PinSW, HIGH);
  attachInterrupt (0, isr, FALLING);

  Serial.begin(9600);


  ads1115.begin();

}

///////////////////////////////////////////
//  _    ___   ___  ___
// | |  / _ \ / _ \| _ \
// | |_| (_) | (_) |  _/
// |____\___/ \___/|_|
//
//////////////////////////////////////////////


void loop()
{




  if (millis() > myTimeout + myTimer ) {
    myTimer = millis();
    timerInterrupt();
    obenlinks = ads1115.readADC_SingleEnded(2);
    obenrechts = ads1115.readADC_SingleEnded(3);
    untenlinks = ads1115.readADC_SingleEnded(0);
    untenrechts = ads1115.readADC_SingleEnded(1);

    if (OutputSerial) {
      Serial.println(obenlinks);  //Außgabe an den seriell monito
      Serial.println(obenrechts);  //Außgabe an den seriell monito
      Serial.println(untenlinks);  //Außgabe an den seriell monito
      Serial.println(untenrechts);  //Außgabe an den seriell monito
      Serial.println(horizontalachse);
      Serial.println(vertikalachse);
      Serial.println(Step_Auto);
    }
  }

  if (TurnDetected)  {

    PrevPosition = RotaryPosition; // Vorherige Position in Variable speichern
    if (rotationdirection) {
      RotaryPosition = RotaryPosition + 1;
    } // Senkt die Position um 1
    else {
      RotaryPosition = RotaryPosition - 1;
    } // Erhöht die Position um 1

    TurnDetected = false;
    // Serial.println(RotaryPosition);

  }


  if (menue == false)
  {
    delay(500);
    lcd.clear();
    lcd.print("Solar-Tracker");
  }
  if (SteigendeFlanke(digitalRead(7)))
  {
    menue = true;

  }



  ////////////Menü Anzeige

  if (menue) // Ins Hauptmenü
  {

    value = analogRead(0);
    if ((yDir > 400) && (yDir < 600)) {
      if ((value > 700) && (!(temphandvalue == 1)))
      {
        tempMenuenummer++;
        delay(400);
      }

      if ((value < 400) && (!(temphandvalue == 1)))
      {
        tempMenuenummer--;
        delay(400);                             //  Menü:______________
        //  1 | Automatik
      }
    }
    //   2 | Erzeugte Energie
    if (tempMenuenummer > 6)                         //   3 | Verbrauchte Energie
      tempMenuenummer = 1;                           //   4 | Wirkungsgrad
    if (tempMenuenummer < 0)                         //   5 | Handbedienung X/Y- Achse
      tempMenuenummer = 6;                           //   6 | Exit

    if (SteigendeFlanke((!digitalRead(7)) && (temphandvalue != 1) && (analogRead(X_pin) < 400) ))
      temphandvalue = 1;
    if (SteigendeFlanke((!digitalRead(7)) && (temphandvalue == 1) && (analogRead(X_pin) > 600) ))
      temphandvalue = 0;



    if ((!(tempMenuenummer == Menuenummer)) || (!(temphandvalue == handvalue)) ) {
      Menuenummer = tempMenuenummer;
      handvalue = temphandvalue;



      if (Menuenummer == 1)
      {
        if (handvalue == 0)
        {
          //  delay(400);
          lcd.clear();
          lcd.setCursor (0, 0); // go to start of first line
          lcd.print("Automatik");
          lcd.setCursor (0, 1); // go to start of 2nd line
          lcd.print("OFF");
          //        if (SteigendeFlanke(!digitalRead(7)) && (handvalue != 1))
          //          handvalue = 1;
        }

        if (handvalue == 1)
        {
          // delay(400);
          lcd.clear();
          lcd.setCursor (0, 0); // go to start of first line
          lcd.print("Automatik");
          lcd.setCursor (0, 1); // go to start of 2nd line
          lcd.print("ON");


          delay(400);

          /////////////////////////////////////////////////////Hier Handbedienung rein!!!!!!!!



          //  if (SteigendeFlanke(digitalRead(7)) && handvalue == 1 && (value > 700))
          //   handvalue = 0;

        }
      }


      else if (Menuenummer == 2)
      {
        lcd.clear();
        lcd.print("Energie erzeugt");
        lcd.setCursor (0, 1); // go to start of 2nd line
        lcd.print(input_wh, 4);
        lcd.print("Wh");
        //   delay(400);
      }


      else if (Menuenummer == 3)
      {
        lcd.clear();
        lcd.print("Energie genutzt");
        lcd.setCursor (0, 1); // go to start of 2nd line
        lcd.print(output_wh, 4);
        lcd.print("Wh");
        // delay(400);
      }


      else if (Menuenummer == 4)
      {
        lcd.clear();
        lcd.setCursor (0, 0); // go to start of first line
        lcd.print("Wirkungsgrad");
        lcd.setCursor (0, 1); // go to start of 2nd line
        lcd.print(wirkungsgrad, 3);
        //delay(400);
      }


      else if (Menuenummer == 5)
      {
        if (handvalue == 0)
        {
          //delay(400);
          lcd.clear();
          lcd.setCursor (0, 0); // go to start of first line
          lcd.print("Handbedienung");
          lcd.setCursor (0, 1); // go to start of 2nd line
          lcd.print("X/Y-Achse");
          //        if (SteigendeFlanke(!digitalRead(7)) && (handvalue != 1))
          //          handvalue = 1;
        }

        if (handvalue == 1)
        {
          ///////////////////////////////////
          //  delay(100);
          /////////////////////////////////////
          lcd.clear();
          lcd.setCursor (0, 0); // go to start of first line
          lcd.print("Handbedienung");
          lcd.setCursor (0, 1); // go to start of 2nd line
          lcd.print("Achsen aktiv");


          delay(400);



          //        if (SteigendeFlanke(digitalRead(7)) && (handvalue == 1) && (value > 700))
          //          handvalue = 0;

        }
      }




      if (Menuenummer == 6)
      {
        lcd.clear();
        lcd.setCursor (0, 0); // go to start of first line
        lcd.print("Exit");
        if (SteigendeFlanke(!digitalRead(7)))
          menue = false;
        // delay(200);
      }
    }
  }

  ///////////////////////Handbetrieb////////////////////////////
  //___  ___                        _  ___  ______________ _____
  //|  \/  |                       | | |  \/  |  _  |  _  \  ___|
  //| .  . | __ _ _ __  _   _  __ _| | | .  . | | | | | | | |__
  //| |\/| |/ _` | '_ \| | | |/ _` | | | |\/| | | | | | | |  __|
  //| |  | | (_| | | | | |_| | (_| | | | |  | \ \_/ / |/ /| |___
  //\_|  |_/\__,_|_| |_|\__,_|\__,_|_| \_|  |_/\___/|___/ \____/
  //
  ///////////////////////////////////////////////////////////////




  if ((handvalue == 1) && Menuenummer == 5 )
  {

    myservo.attach(3);

    xDir = analogRead(X_pin);
    yDir = analogRead(Y_pin);

    if ((xDir < 400) && (digitalRead(7))) {
      pos = pos + 2; //((xDir-500)/100);
      delay(100);
      if (pos > 180) {
        pos = 180;
      }
    }
    if ((xDir > 600) && (digitalRead(7))) {
      pos = pos - 2; //((500-xDir)/100);
      delay(100);
      if (pos < 0) {
        pos = 0;
      }
    }

    myservo.write(pos);

    //Y_Kordinate DC Motor Antrieb Drehachse Panel

    if (analogRead(Y_pin) < 400) {
      digitalWrite(PIN_EnableMotor, HIGH);
      delay(10);
      analogWrite(PIN_Linksauf, 100);
      analogWrite(PIN_Rechtslauf, 0);
      rotationdirection = true;


    }
    if (analogRead(Y_pin) > 600) {
      digitalWrite(PIN_EnableMotor, HIGH);
      delay(10);
      analogWrite(PIN_Rechtslauf, 100);
      analogWrite(PIN_Linksauf, 0);
      rotationdirection = false;

    }
    if ((yDir > 400) && (yDir < 600)) {
      analogWrite(PIN_Linksauf, 0);
      analogWrite(PIN_Rechtslauf, 0);
      digitalWrite(PIN_EnableMotor, LOW);
      rotationdirection = false;
    }
  }
  ////////////////////////////Automatik///////////////////////////////

  //  ___        _                        _   _       ___  ______________ _____
  // / _ \      | |                      | | (_)      |  \/  |  _  |  _  \  ___|
  /// /_\ \_   _| |_ ___  _ __ ___   __ _| |_ _  ___  | .  . | | | | | | | |__
  //|  _  | | | | __/ _ \| '_ ` _ \ / _` | __| |/ __| | |\/| | | | | | | |  __|
  //| | | | |_| | || (_) | | | | | | (_| | |_| | (__  | |  | \ \_/ / |/ /| |___
  //\_| |_/\__,_|\__\___/|_| |_| |_|\__,_|\__|_|\___| \_|  |_/\___/|___/ \____/

  ///////////////////////////////////////////////////////////////////



  if ((handvalue == 1) && (Menuenummer == 1) )
  {
    myservo.attach(3);

    obenlinks = ads1115.readADC_SingleEnded(2);
    obenrechts = ads1115.readADC_SingleEnded(3);
    untenlinks = ads1115.readADC_SingleEnded(1);//- 2096);
    untenrechts = ads1115.readADC_SingleEnded(0);//- 80);

    if (OutputSerial) {
      Serial.println(obenlinks);  //Außgabe an den seriell monito
      Serial.println(obenrechts);  //Außgabe an den seriell monito
      Serial.println(untenlinks);  //Außgabe an den seriell monito
      Serial.println(untenrechts);  //Außgabe an den seriell monito
      Serial.println(horizontalachse);
      Serial.println(vertikalachse);
      Serial.println(Step_Auto);
    }

    switch (Step_Auto) {
      case 0:

        if (OutputSerial) {
          Serial.println(obenlinks);  //Außgabe an den seriell monito
          Serial.println(obenrechts);  //Außgabe an den seriell monito
          Serial.println(untenlinks);  //Außgabe an den seriell monito
          Serial.println(untenrechts);  //Außgabe an den seriell monito
          Serial.println(horizontalachse);
          Serial.println(vertikalachse);
          Serial.println(Step_Auto);
        }
        Step_Auto = 10;
        break;


      case 10:
        if ((obenlinks > untenlinks) || (obenrechts > untenrechts))
          Step_Auto = 20;
        else if ((obenlinks < untenlinks) || (obenrechts < untenrechts))
          Step_Auto = 30;

        if (OutputSerial) {
          Serial.println(Step_Auto);
        }
        break;

      case 20:
        if ((obenlinks) > (obenrechts + toleranzwert))
          horizontalachse = 100; //links drehen
        else if ((obenrechts ) > (obenlinks + toleranzwert))
          horizontalachse = -100; //rechts drehen
        else
          horizontalachse = 0;

        if (horizontalachse == 0)
          Step_Auto = 40;



        if (OutputSerial) {
          Serial.println(Step_Auto);
        }
        break;

      case 30:
        if ((untenlinks) > (untenrechts + toleranzwert))
          horizontalachse = 100; //links drehen
        else if ((untenrechts ) > (untenlinks + toleranzwert))
          horizontalachse = -100; //rechts drehen
        else
          horizontalachse = 0;

        if (horizontalachse == 0)
          Step_Auto = 40;


        if (OutputSerial) {
          Serial.println(Step_Auto);
        }
        break;

      case 40:
        if (untenlinks > (obenlinks + toleranzwert))
          vertikalachse = -100; //links drehen
        else if (obenlinks  > (untenlinks + toleranzwert))
          vertikalachse = 100; //rechts drehen
        else
          vertikalachse = 0;

        if (vertikalachse == 0)
          Step_Auto = 60;


        if (OutputSerial) {
          Serial.println(Step_Auto);
          Serial.println(obenlinks);  //Außgabe an den seriell monito
          Serial.println(obenrechts);  //Außgabe an den seriell monito
          Serial.println(untenlinks);  //Außgabe an den seriell monito
          Serial.println(untenrechts);  //Außgabe an den seriell monito
          Serial.println(horizontalachse);
          Serial.println(vertikalachse);
        }
        break;

      case 50:
        if (untenrechts > (obenrechts + toleranzwert))
          vertikalachse = -100; //links drehen
        else if (obenrechts  > (untenrechts + toleranzwert))
          vertikalachse = 100; //rechts drehen
        else
          vertikalachse = 0;
        if (vertikalachse == 0)
          Step_Auto = 60;
          
      case 60:
        Serial.println("pause");
        


        if (millis() > AutoTimeout + AutoTimer ) {
          AutoTimer = millis();
          Step_Auto = 0;

          if (OutputSerial) {
            Serial.println(obenlinks);  //Außgabe an den seriell monito
            Serial.println(obenrechts);  //Außgabe an den seriell monito
            Serial.println(untenlinks);  //Außgabe an den seriell monito
            Serial.println(untenrechts);  //Außgabe an den seriell monito
            Serial.println(horizontalachse);
            Serial.println(vertikalachse);
            Serial.println(Step_Auto);
          }
        }

        break;

      default: Serial.println("error"); break;
    }






    if ((horizontalachse) < -50) {
      pos = pos + 1; //((xDir-500)/100);
      //delay(10);
      if (pos > 180) {
        pos = 180;
      }
    }
    if ((horizontalachse) > 50) {
      pos = pos - 1; //((500-xDir)/100);
      //delay(10);
      if (pos < 0) {
        pos = 0;
      }
    }
    myservo.write(pos);
    //
    //
    if ((vertikalachse > 50)) {//&& (RotaryPosition < 0 )
      digitalWrite(PIN_EnableMotor, HIGH);
      delay(10);
      analogWrite(PIN_Linksauf, 75);
      analogWrite(PIN_Rechtslauf, 0);
      rotationdirection = true;
    }
    if ((vertikalachse < -50)) {//&& (RotaryPosition > 15)
      digitalWrite(PIN_EnableMotor, HIGH);
      delay(10);
      analogWrite(PIN_Rechtslauf, 75);
      analogWrite(PIN_Linksauf, 0);
      rotationdirection = false;
    }
    if ((vertikalachse == 0)) { //||(Step_Auto<35)((vertikalachse = 0)||(RotaryPosition > 15)||(RotaryPosition < 0 )) {
      analogWrite(PIN_Linksauf, 0);
      analogWrite(PIN_Rechtslauf, 0);
      digitalWrite(PIN_EnableMotor, LOW);
      rotationdirection = false;
    }

    //
    //
    //
  }
}
