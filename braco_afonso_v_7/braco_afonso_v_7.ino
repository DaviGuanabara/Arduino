
#include <Servo.h>

//============================================================
//        Declaring variables
//============================================================

//Servo Configurations
static const int BASE_Z = 0;
static const int BASE_Y = 1;
static const int BRACO_Y = 2;
static const int SERVO_DELAY = 80;
static const int SERVO_ID_BIAS = 4;
static const int SERVO_INIT_POS = 90;
static const int SERVO_POS_BIAS = SERVO_INIT_POS;

//Intantiate servo
Servo servos[3];
int servos_pos[3];

//Arm sizes and initial angles
static const double A_Um = 16;
static const double A_Zero = 11;

// Enable pins
static const int ENABLE_TRAINING_PIN = 3;



//============================================================
//        Main Methods
//============================================================


void setup() { // Open serial communications and wait for port to open:
  initSerial();
  userIntro();
  initServos();
  kinematicsDemo();
}

void loop() {

  //int enable_traning = digitalRead(ENABLE_TRAINING_PIN);

  String movs = "";
  movs = getCommands(digitalRead(ENABLE_TRAINING_PIN));
  if(movs.length()>0){
    Serial.println("movs: ");
    Serial.println(movs);
    executeMovs(movs);
  }
  
}

//============================================================
//        Second line Methods
//============================================================


String getCommands(int enable_traning) {
  //int enable_traning = digitalRead(ENABLE_TRAINING_PIN);
  if (enable_traning == HIGH) {
    return traningMode();
  } else if(Serial.available()>0){
    return receiveData();
  } else return "";
}

String receiveData() {
  String inString = "";
  while (Serial.available() > 0) {
    inString += (char) Serial.read();
    Serial.println(inString);
    delay(5);
  }
  return inString;
}


void executeMovs(String movs) {

  int servo_id = -1;
  boolean isNegative;
  String inString = "";
  
  for (int i = 0; i < movs.length(); i++) {
    
    int inChar = movs[i];
    
    if (isDigit(inChar)) {
      inString += (char)inChar;

    } else if (inChar == '-') {
      isNegative = true;

    } else if (inChar == '+') {
      isNegative = false;

    } else if (inChar == '<') {
      // identify motor
      servo_id  = inString.toInt();
      inString = "";

    } else if (inChar == '|') {

      int number = (int) inString.toInt();
      if (isNegative) {
        number = -1 * number;
      }
      
      moveServo(servo_id, number);
      //servos[servo_id].write((int) inString.toInt());
      inString = "";

    } else if (inChar == '\n') {
      inString = "";

      Serial.println("finalizou");
    }

  }
}

//============================================================
//        Operations
//============================================================

String traningMode() {

  boolean enable = (boolean)digitalRead(3);
  if (enable == true) {
    Serial.println("Traning mode enable");
  }
  
  String movs;
  int potpos;
  int last_mov[3];
  int potpin = 4;

  while (enable) {
    for (int i = potpin; i < potpin + 3; i++) {

      potpos = map(analogRead(i), 0, 1023, -90, 90);

      if (!areThisEqual(potpos, last_mov[i - potpin])) {
        int servo_id = i - potpin;
        movs = addMotor(servo_id, potpos, movs);
        Serial.println("mov detected: ");
        Serial.println(potpos);
        last_mov[servo_id] = potpos;
        servos[servo_id].write(potpos);
        delay(25);
      }


    }
    enable = (boolean)digitalRead(ENABLE_TRAINING_PIN);
  }
  return movs;
}

String addMotor(int servo_id, int potpos, String mov) {

  mov.concat(String(servo_id));
  mov.concat("<");

  if (potpos < 0) {
    potpos = -1 * potpos;
    mov.concat("-");
    mov.concat(String(potpos));

  } else {
    mov.concat("+");
    mov.concat(String(potpos));
  }

  mov.concat("|");

  return mov;
}

//============================================================
//        Setup Methods
//============================================================

void initSerial() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}



void kinematicsDemo() {
  double theta[3];

  Serial.println("");
  Serial.println("Kinematics Demo: ");

  for (int i = 0; i < 3; i++) {
    theta[i] = 0;
  }

  printAngles(theta);

  double mat[4][4];
  double resultado[4][4];
  identity(&mat[0][0]);

  Serial.println("");
  Serial.println("Foward kinematic: ");

  forwardKinematic(theta, A_Zero, A_Um, &mat[0][0], &resultado[0][0]);
  printMatrix(resultado);

  Serial.println("");
  Serial.println("Inverse kinematic: ");

//  double x = resultado[0][3];
//  double y = resultado[1][3];
//  double z = resultado[2][3];

  printPoint(resultado[0][3],resultado[1][3],resultado[2][3]);
  
  inverseKinematic(resultado[0][3],resultado[1][3],resultado[2][3], &theta[0]);
  
  printAngles(theta);
}

void initServos() {

  for ( int servo_id = BASE_Z; servo_id < BRACO_Y + 1; servo_id++) {
    servos[servo_id].attach(SERVO_ID_BIAS + servo_id);
    servos_pos[servo_id] = SERVO_INIT_POS;
  }

}

//============================================================
//         Servo Moviment Control and methods
//============================================================

void moveServo(int servo_id, int servo_pos) {
  printServoMoviment(servo_id, servo_pos);
  servos[servo_id].write(servos_pos[servo_id] + SERVO_POS_BIAS);
  delay(SERVO_DELAY);
}

//void moveServos() {
//
//  for ( int servo_id = BASE_Z; servo_id < BRACO_Y + 1; servo_id++) {
//
//    servos[servo_id].write(servos_pos[servo_id] + SERVO_POS_BIAS);
//  }
//}

void savePos(int servo_id, int pos) {
  servos_pos[servo_id] = pos;
}


//============================================================
//         Utils
//============================================================

//with filter
boolean areThisEqual(int a, int b) {

  if (b < a - 8 || b > a + 8) {
    return false;
  } else return true;
}

//============================================================
//         Kinematics
//============================================================

/*
                       a1 = 16 cm
               (rotY) .__________
                      |
                      |
   a0 = 11 cm         |
                      |
                      .(rotZ&rotY)


  M02  =  RotationZ(theta0 + 0)*RotationY(theta1 + 0)*TranslationZ(A0)*RotationY(theta2 + 90)*TranslationZ(A1)

*/


void inverseKinematic(double x, double y, double z, double *theta) {

  static const int A_Zero = 11;
  static const int A_Um = 16;

  for (int i = 0; i < 3; i++) {
    theta[i] = 0;
  }

  double w    = sqrt(pow(x, 2) + pow(y, 2));
  double gama = sqrt(pow(w, 2) + pow(z, 2));
  double alfa = acos((pow(A_Um, 2) - pow(gama, 2) - pow(A_Zero, 2)) / (-2 * gama * A_Zero));

  theta[0] = atan(y / x) * 360 / (2 * M_PI);
  theta[1] = (acos(z / gama) - alfa) * 360 / (2 * M_PI);
  theta[2] = (-1 * acos((pow(gama, 2) - pow(A_Zero, 2) - pow(A_Um, 2)) / (-2 * A_Zero * A_Um)) + M_PI / 2) * 360 / (2 * M_PI);

}





void forwardKinematic(double theta[3], double A_Zero, double A_Um, double *mat, double *result)
{
  static const double theta0_bias = 0;
  static const double theta1_bias = 0;
  static const double theta2_bias = 90;

  theta[0] = (theta[0] + theta0_bias) * (2 * M_PI / 360);
  theta[1] = (theta[1] + theta1_bias) * (2 * M_PI / 360);
  theta[2] = (theta[2] + theta2_bias) * (2 * M_PI / 360);

  double mat1[4][4];
  rotateZ(mat, theta[0], &mat1[0][0]);
  double mat2[4][4];
  rotateY(&mat1[0][0], theta[1], &mat2[0][0]);
  double mat3[4][4];
  translateZ(&mat2[0][0], A_Zero, &mat3[0][0]);
  double mat4[4][4];
  rotateY(&mat3[0][0], theta[2], &mat4[0][0]);
  double mat5[4][4];
  translateZ(&mat4[0][0], A_Um, &mat5[0][0]);

  equalize(&mat5[0][0], result);
}




//Rotation methods
void rotateY(double *base, double theta, double *result) {

  double rotY[4][4] = {{ cos(theta), 0, sin(theta), 0},
    {0, 1, 0, 0},
    { -sin(theta), 0, cos(theta), 0},
    {0, 0, 0, 1}
  };

  mMatriz(base, &rotY[0][0], result);

}

void rotateZ(double *base, double theta, double *result) {

  double rotZ[4][4] = {{cos(theta), -sin(theta), 0, 0},
    {sin(theta), cos(theta), 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };

  mMatriz(base, &rotZ[0][0], result);
}

void translateZ(double *base, double translation, double *result) {

  double transMatrix[4][4] =  {{1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, translation},
    {0, 0, 0, 1}
  };

  mMatriz(base, &transMatrix[0][0], result);

}

void mMatriz(double *a, double *b, double *result) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      result[i * 4 + j] = 0;
      for (int k = 0; k < 4; k++)
        result[i * 4 + j] = result[i * 4 + j] + a[i * 4 + k] * b[k * 4 + j];
    }
  }
}

void identityMatrix(double *mat) {
  for (int t = 0; t < 4; t++) {
    mat[t * 4 + t] = 1;
  }
}

void equalize (double *p, double *q)
{
  q = p;
}

void equalize (char *p, char *q)
{
  q = p;
}

void printMatrix(double m[4][4]) {
  for (int i = 0; i <= 3; i++) {
    Serial.print("[ ");
    for (int j = 0; j <= 3; j++) {
      Serial.print(m[i][j]);
      if (j < 3) Serial.print(" , ");
      else Serial.println(" ]");
    }
  }
}

void identity(double *mat)
{
  int row, col;
  int num = 4;

  for (row = 0; row < num; row++)
  {
    for (col = 0; col < num; col++)
    {
      // Checking if row is equal to column
      if (row == col)
        mat[row * num + col] = 1;
      else
        mat[row * num + col] = 0;
    }
  }
}

//============================================================
//        Print Methods
//============================================================

void userIntro() {
  Serial.println("");
  Serial.println("Davi Guanabara - Rodrigo Aguiar ");
  Serial.println("Braço robótico - perna do exapod");
  Serial.println("                                                                                                                    ");
  Serial.println("                                                                                                                    ");
  Serial.println("                                                                                                                    ");
  Serial.println("                                                                                                                    ");
  Serial.println("                     a1 = 16 cm                                                                                     ");
  Serial.println("             (rotY) .__________                                                                                     ");
  Serial.println("                    |                                                                                               ");
  Serial.println("                    |                                                                                               ");
  Serial.println(" a0 = 11 cm         |                                                                                               ");
  Serial.println("                    |                                                                                               ");
  Serial.println("                    .(rotZ&rotY)                                                                                    ");
  Serial.println("                                                                                                                    ");
  Serial.println("                                                                                                                    ");
  Serial.println("M02  =  RotationZ(theta0 + 0)*RotationY(theta1 + 0)*TranslationZ(A0)*RotationY(theta2 + 90)*TranslationZ(A1)        ");
  Serial.println("                                                                                                                    ");
  Serial.println("Para o controle direto dos motores, escreva: id do motor<posição do motor|                                          ");
  Serial.println("exemplo: 0<-80|1<-90|2<+70|                                                                                             ");
  Serial.println("");
}

void printPoint(double x, double y, double z){
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.println(z);
  Serial.println("");
}

void printServoMoviment(int servo_id, int angle ){
      Serial.print("Servo_id: ");
      Serial.println(servo_id);
      Serial.print("Angulo do motor: ");
      Serial.println(angle);
}

void printAngles(double theta[3]){
  for (int i = 0; i < 3; i++) {
    Serial.print("Theta");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(theta[i]);
  }
}
