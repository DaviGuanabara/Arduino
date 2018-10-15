#include <math.h>





int b[3];

boolean done = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("ativado");

}

void loop() {
  // put your main code here, to run repeatedly:
  double in[3];
  int a=0;
  char inChar;
  String inString = "";
  

  //receiveData(in);
  

  in[0] = 21.5;
  in[1] = 0;
  in[2] = 5.5;

  Serial.print("in[0] = ");
  Serial.println(in[0]);
  Serial.print("in[1] = ");
  Serial.println(in[1]);
  Serial.print("in[2] = ");
  Serial.println(in[2]);
  cinematicaInversa(in[0],in[1],in[2]);

  in[0] = 10;
  in[1] = 10;
  in[2] = 10;
    Serial.print("in[0] = ");
  Serial.println(in[0]);
  Serial.print("in[1] = ");
  Serial.println(in[1]);
  Serial.print("in[2] = ");
  Serial.println(in[2]);
  cinematicaInversa(in[0],in[1],in[2]);

  in[0] = 27;
  in[1] = 0; 
  in[2] = 0;
  Serial.print("in[0] = ");
  Serial.println(in[0]);
  Serial.print("in[1] = ");
  Serial.println(in[1]);
  Serial.print("in[2] = ");
  Serial.println(in[2]);
  cinematicaInversa(in[0],in[1],in[2]);

  in[0] = 5.5;
  in[1] = 0; 
  in[2] = 13.5;
  Serial.print("in[0] = ");
  Serial.println(in[0]);
  Serial.print("in[1] = ");
  Serial.println(in[1]);
  Serial.print("in[2] = ");
  Serial.println(in[2]);
  cinematicaInversa(in[0],in[1],in[2]);

  in[0] = 21.5;
  in[1] = 0; 
  in[2] = 9.52627944163;
  Serial.print("in[0] = ");
  Serial.println(in[0]);
  Serial.print("in[1] = ");
  Serial.println(in[1]);
  Serial.print("in[2] = ");
  Serial.println(in[2]);
  cinematicaInversa(in[0],in[1],in[2]);

  Serial.println("Finalizou!");
  while(true){
    ;
  }
  

//
//
//  if(  b[0] != in[0]||
//  b[2] != in[1]||
//  b[1] != in[2]){
//     cinematicaInversa((float)in[0],(float)in[1],(float)in[2]);
//  }
//
//
// 
//  b[0] = in[0];
//  b[2] = in[1];
//  b[1] = in[2];
  

}




void cinematicaInversa(double x,double y,double z){

  /*
 *                      a1 = 16 cm
 *              (rotY) .__________
 *                     |
 *                     | 
 *  a0 = 11 cm         |
 *                     |
 *                     .(rotZ&rotY)
 *                     
 *                     
 *             RotZ*RotY*TransA0*RotY*TransA1        
 *                     
 */

  static const int A_Zero = 11;
  static const int A_Um = 16;
  static const float Pi = 3.14159; 
  static const double m_PI = 3.14159265358979323846;

  double w = sqrt(pow(x,2)+pow(y,2));
  
  Serial.println("w: ");
  Serial.println(w);

  double gama = sqrt(pow(w,2)+pow(z,2));
  
  Serial.println("gama: ");
  Serial.println(gama);
  double alfa = acos((pow(A_Um,2) - pow(gama,2) - pow(A_Zero, 2))/(-2*gama*A_Zero));
  
  Serial.println("alfa: ");
  Serial.println(alfa);
  Serial.print("miolo do alfa: ");
  Serial.println((pow(A_Um,2) - pow(gama,2) - pow(A_Zero, 2))/(-2*gama*A_Zero));
  double teta1 = acos(z/gama) - alfa;
  Serial.println("teta1: ");
  Serial.println(teta1*360/(2*m_PI));
  double teta2 = -1*acos((pow(gama,2) - pow(A_Zero,2) - pow(A_Um,2))/(-2*A_Zero*A_Um)) + m_PI/2;
  Serial.println("teta2: ");
  Serial.println(teta2*360/(2*m_PI));
  double teta0 = atan(y/x)*360/(2*m_PI);


 
    Serial.println("teta0: ");
  Serial.println(teta0);
}

void receiveData(int data[]){
  //int data[3];
  String inString = "";
  int dataIndex = 0;
  
  while(Serial.available()>0){
   int inChar = Serial.read();


    if(isDigit(inChar)){
      inString += (char)inChar;
      Serial.println("inString: ");
      Serial.println(inString);
    
   } else if (inChar == '|') {
        data[dataIndex] = ((int)inString.toInt());//*2*Pi/360;
        Serial.print("Valor indexado: ");
        Serial.println(data[dataIndex]);
      dataIndex++;
      inString = "";
   
    
    } else if (inChar == '\n') {
      inString = "";

      Serial.println("finalizou");
    }
   }
   //return data;
}
