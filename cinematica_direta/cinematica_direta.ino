#include <Geometry.h>

float A_Zero = 11;
float A_Um = 16;

static const int theta1 = 0;
static const int theta0 = 0;
static const int theta2 = 90;

void setup() {
  // put your setup code here, to run once:


Serial.begin(9600);
Transformation base_frame;
base_frame.RotateZ(theta0*2*M_PI/360);
base_frame.RotateY(theta1*2*M_PI/360);
base_frame.Translate(0,0,11.0);
base_frame.RotateY(theta2*2*M_PI/360);
base_frame.Translate(22.1, 56.43, 100.54);
Serial << "base_frame = " <<  base_frame << "\n";

}

void loop() {
  // put your main code here, to run repeatedly:

}

Transformation initFrame(double theta0, double theta1, double theta2, float A_Zero, float A_Um){

//  static const int A_Zero = 11;
//  static const int A_Um = 16;
//
//  static const int theta1 = 0;
//  static const int teta0 = 0;
//  static const int theta2 = 90*2*M_PI/360;

  
  Transformation base_frame;
  base_frame.RotateZ(theta0*2*M_PI/360);
  base_frame.RotateY(theta1*2*M_PI/360);
  base_frame.Translate(0,0,A_Zero);
  base_frame.RotateY(theta2*2*M_PI/360);
  base_frame.Translate(0,0,A_Um);
  return base_frame;
}
