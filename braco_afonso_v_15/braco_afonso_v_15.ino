
#include <Servo.h>


  /*
   * Davi Guanabara - Robótica Industrial Unifor
   * 
   * Não sei pq porra de motivo a funcao forwardKinematic(), se usado abaixo, da pau pra sempre!!! 
   * Problemas foram identificados na funcao isAnglesForbbiden, generateLine, getCommandsToWriteWord
   * 
   */
   /*
 * parametric equation of a circle
 * x = cx + r * cos(a)
 * y = cy + r * sin(a)
 * Where r is the RADIUS, cx,cy the origin, and a the angle.
 */

//============================================================
//        Declaring variables
//============================================================

// letter order
static const int Letter_A = 0;
static const int Letter_B = 1;
static const int Letter_C = 2;
static const int Letter_D = 3;
static const int Letter_E = 4;
static const int Letter_F = 5;
static const int Letter_G = 6;
static const int Letter_H = 7;
static const int Letter_I = 8;
static const int Letter_J = 9;
static const int Letter_K = 10;
static const int Letter_L = 11;
static const int Letter_M = 12;
static const int Letter_N = 13;
static const int Letter_O = 14;
static const int Letter_P = 15;
static const int Letter_Q = 16;
static const int Letter_R = 17;
static const int Letter_S = 18;
static const int Letter_T = 19;
static const int Letter_U = 20;
static const int Letter_V = 21;
static const int Letter_X = 22;
static const int Letter_Y = 23;
static const int Letter_Z = 24;


//Writing Configurations
static const int BASE_RADIUS_TO_WRITE = 10;
static const int POSITION_OF_Z_OF_GROUND = 0;
static const int LENGTH_LETTER_MATRIX_TO_WRITE = 3; 
static const int DEEPTH_LETTER_MATRIX_TO_WRITE = 3;
static const int DISTANCE_BETWEEN_DOTS_TO_WRITE = 1;
static const int NUMBER_OF_NO_LETTERS_AT_ENDING_TO_WRITE = 1;
static const int NUMBER_OF_NO_LETTERS_AT_BEGINING_TO_WRITE = 1;
static const int NUMBER_OF_NO_LETTERS = NUMBER_OF_NO_LETTERS_AT_BEGINING_TO_WRITE + NUMBER_OF_NO_LETTERS_AT_ENDING_TO_WRITE;
static const int TOP_RADIUS_TO_WRITE = DEEPTH_LETTER_MATRIX_TO_WRITE + BASE_RADIUS_TO_WRITE;
static const int MAXIMUM_LETTERS_TO_WRITE = (int) ((M_PI*BASE_RADIUS_TO_WRITE/(DISTANCE_BETWEEN_DOTS_TO_WRITE*(LENGTH_LETTER_MATRIX_TO_WRITE + 1))) - NUMBER_OF_NO_LETTERS); // 1 POR PREUCAUCAO  9;

static const int MAXIMUM_ANGULATION = 90;
static const int MAXIMUM_ABSOLUTE_ANGULATION = 180;
static const int ANGLE_BETWEEN_LETTERS = MAXIMUM_ABSOLUTE_ANGULATION/(MAXIMUM_LETTERS_TO_WRITE + NUMBER_OF_NO_LETTERS);

// conversions
static const double GRAUS_TO_RADIANS = 2*M_PI/360;
static const double RADIANS_TO_GRAUS = 360/2*M_PI;

// Enable pins
static const int RECEIVER_DELAY = 1;
static const int ENABLE_TRAINING_PIN = 3;

//Arm sizes
static const double A_Um = 16;
static const double A_Zero = 11;

//Servo Configurations
static const int BASE_Z = 0;
static const int BASE_Y = 1;
static const int BRACO_Y = 2;

static const int SERVO_DELAY = 80;
static const int SERVO_ID_BIAS = 4;
static const int SERVO_INIT_POS = 0;
static const int SERVO_POS_BIAS = 90;

static const double Theta0_BIAS = 0;
static const double Theta1_BIAS = 0;
static const double Theta2_BIAS = 90;

static const int FILTER_FOR_POTENTIOMETER = 8;

static const int MINIMUM_DISTANCE_ALLOWED_TO_GROUND = POSITION_OF_Z_OF_GROUND - 1;
static const int MINIMUM_DISTANCE_ALLOWED_TO_ORIGIN = 6;

//Intantiate servo
Servo servos[3];
double currentServoAngles[3];

//String of words
String words[25];


//============================================================
//        Main Methods
//============================================================


void setup() { // Open serial communications and wait for port to open:
  initSerial();
  userIntro();
  initServos();
  initWords();
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
  while(true){
    ;
  }
}



//============================================================
//        Second line Methods
//============================================================


String getCommands(int enable_traning) {
  
  if (enable_traning == HIGH) {
    return traningMode();
    
  } else { 

    //if(Serial.available()>0){
      
//    double point0[2];
//    point0[0] = 5;
//    point0[1] = 0;
//  
//    double point1[2];
//    point1[0] = 10;
//    point1[1] = 0;
//    
//    String movs = "";
//    generateLine(point0, point1, movs);
//    return movs;


    return getCommandsToWriteWord("davi");
    //return receiveData(); 
    }// else 
    return "";
  //} 
}

String receiveData() {
  String inString = "";
  while (Serial.available() > 0) {
    inString += (char) Serial.read();
    Serial.println(inString);
    delay(RECEIVER_DELAY);
  }
  return inString;
}


void executeMovs(String movs) {

  int servo_id = -1;
  boolean isNegative;
  String inString = "";

  Serial.println("Executing moves.");
  
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
      //isAngleForbbiden(servo_id, number);
      //servos[servo_id].write((int) inString.toInt());
      boolean a = isAngleForbbiden(servo_id, number);
      
      //Serial.println("Is Angle forbbiden?: ");
      Serial.print(a);
      inString = "";

    } else if (inChar == '\n') {
      inString = "";

      
    }

  }
  Serial.println("Finalizing execution");
}

//============================================================
//        Operations
//============================================================

String traningMode() {

  boolean enable = (boolean)digitalRead(ENABLE_TRAINING_PIN);
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
  Serial.println("Fim do treinamento.");
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

  forwardKinematic(theta, &mat[0][0], &resultado[0][0]);
  printMatrix(resultado);

  Serial.println("");
  Serial.println("Inverse kinematic: ");

  printPoint(resultado[0][3],resultado[1][3],resultado[2][3]);
  
  inverseKinematic(resultado[0][3],resultado[1][3],resultado[2][3], &theta[0]);
  
  printAngles(theta);
}

void initServos() {

  for ( int servo_id = BASE_Z; servo_id < BRACO_Y + 1; servo_id++) {
    servos[servo_id].attach(servo_id + SERVO_ID_BIAS);
    //servos_pos[servo_id] = SERVO_INIT_POS;
    currentServoAngles[servo_id] = SERVO_INIT_POS;
    moveServo(servo_id, SERVO_INIT_POS);
  }
}  


void initWords(){
  
  words[Letter_A] = "0|1|2|5|8|5|4|3|6|";
  words[Letter_B] = "0|3|6|7|5|4|1|0|";
  words[Letter_C] = "1|0|3|6|7|";
  words[Letter_D] = "0|1|5|7|6|3|0|";
  words[Letter_E] = "1|0|3|4|3|6|7|";
  words[Letter_F] = "1|0|3|4|3|6|";
  words[Letter_G] = "1|0|3|6|7|5|4|";
  words[Letter_H] = "0|3|6|3|4|5|2|5|8|";
  words[Letter_I] = "0|1|2|1|4|7|6|7|8|";
  words[Letter_J] = "1|4|7|6|"; 
  words[Letter_K] = "0|3|6|3|1|3|7";
  words[Letter_L] = "0|3|6|7|";
  words[Letter_M] = "6|3|0|7|2|5|8|";
  words[Letter_N] = "7|4|1|8|5|2|";
  words[Letter_O] = "0|3|6|7|4|1|0|"; 
  words[Letter_P] = "3|4|1|0|3|6|";
  words[Letter_Q] = "0|3|6|7|8|5|2|1|4|";
  words[Letter_R] = "3|4|1|0|3|6|3|7|";
  words[Letter_S] = "1|0|3|4|7|6|";
  words[Letter_T] = "0|1|2|1|4|7|"; 
  words[Letter_U] = "0|3|6|7|4|1|";
  words[Letter_V] = "0|7|2|";
  words[Letter_X] = "0|4|8|4|2|4|6|";
  words[Letter_Y] = "0|4|1|4|7|";
  words[Letter_Z] = "0|1|2|4|6|7|8|"; 
}  


//============================================================
//         Servo Moviment Control and methods
//============================================================

void moveServo(int servo_id, int servo_pos) {
  //if(!isAngleForbbiden(servo_id,servo_pos)){
    
    //printServoMoviment(servo_id, servo_pos);
    currentServoAngles[servo_id] = servo_pos;
    servos[servo_id].write(servo_pos + SERVO_POS_BIAS);
    delay(SERVO_DELAY);
//
//    if(!isAngleForbbiden(servo_id,servo_pos)){
//      isAngleForbbiden(servo_id, servo_pos);
//    } else {
//    Serial.println("Angle forbbiden");
//  }
}

//============================================================
//         Utils
//============================================================

//with filter

boolean areThisEqual(int a, int b) {

  if (b < a - FILTER_FOR_POTENTIOMETER || b > a + FILTER_FOR_POTENTIOMETER) {
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


void forwardKinematic(double theta[3], double *mat, double *result)
{
  theta[0] = (theta[0] + Theta0_BIAS) * (2 * M_PI / 360);
  theta[1] = (theta[1] + Theta1_BIAS) * (2 * M_PI / 360);
  theta[2] = (theta[2] + Theta2_BIAS) * (2 * M_PI / 360);

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
                       {-sin(theta), 0, cos(theta), 0},
                       {0, 0, 0, 1}
  };

  mMatriz(base, &rotY[0][0], result);

}

void rotateZ(double *base, double theta, double *result) {

  double rotZ[4][4] = {{cos(theta), -sin(theta), 0, 0},
                       { sin(theta), cos(theta), 0, 0},
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





//=====================================================================================================================================================================
//        Moviments Blocked
//=====================================================================================================================================================================


boolean isGround(double x, double y, double z){
  if(z < MINIMUM_DISTANCE_ALLOWED_TO_GROUND) return true;
  else return false;
}

boolean isBase(double x, double y, double z){
  if(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) < MINIMUM_DISTANCE_ALLOWED_TO_ORIGIN) return true;
  else return false;
}

boolean isPositionForbbiden(double x, double y, double z){
  if(isBase(x,y,z) || isGround(x,y,z)) return true;
  else return false;
  
}

boolean isAnglesForbbiden(double theta[3]){

  double mat[4][4];
  double resultado[4][4];

  for(int i=0; i<4; i++){
    for(int j=0; j<4;j++){
      resultado[i][j] = 0;
    }
  }
  /*
   * Não sei pq porra de motivo a funcao forwardKinematic(), se usado abaixo, da pau pra sempre!!! 
   * o que caralhos acontece ??? eu n sei tomanucu
   * 
   * daí, só joguei o que tinha dentro da função aqui... o que mostra que c e ponteiro é uma merda
   */
  identity(&mat[0][0]);

  theta[0] = (theta[0] + Theta0_BIAS) * GRAUS_TO_RADIANS;
  theta[1] = (theta[1] + Theta1_BIAS) * GRAUS_TO_RADIANS;
  theta[2] = (theta[2] + Theta2_BIAS) * GRAUS_TO_RADIANS;

  double mat1[4][4];
  rotateZ(&mat[0][0], theta[0], &mat1[0][0]);
  double mat2[4][4];
  rotateY(&mat1[0][0], theta[1], &mat2[0][0]);
  double mat3[4][4];
  translateZ(&mat2[0][0], A_Zero, &mat3[0][0]);
  double mat4[4][4];
  rotateY(&mat3[0][0], theta[2], &mat4[0][0]);
  double mat5[4][4];
  translateZ(&mat4[0][0], A_Um, &mat5[0][0]);

  return isPositionForbbiden(mat5[0][3],mat5[1][3],mat5[2][3]);
}

boolean isAngleForbbiden(int servo, double angle){

  double angles[3];
  
  angles[0] = currentServoAngles[0];
  angles[1] = currentServoAngles[1];
  angles[2] = currentServoAngles[2];
  angles[servo] = currentServoAngles[servo];
  
  return isAnglesForbbiden(angles);
}


//=====================================================================================================================================================================
//        Geometry
//=====================================================================================================================================================================



void generateLine(double point0[2],double point1[2], String movs){ //, double *result){

  double deltaY = point1[1] - point0[1];
  double deltaX = point1[0] - point0[0];

  double distance = sqrt(pow(deltaY,2) + pow(deltaX,2));

  double cosTriangle = deltaX/distance;
  double sinTriangle = deltaY/distance;
  
  int discreteDistance = (int) distance;

  double lineStep = distance/discreteDistance;

  double pointTemp[2];
  double pointList[discreteDistance][2];

   /*
   * Gera os pontos da reta. O método abaixo pode ser transformado em um método
   */
  for(int i = 0; i < discreteDistance + 1; i++){
    pointList[i][0] = 0;
    pointList[i][1] = 0;

    if(i == 0){
      pointList[i][0] = point0[0];
      pointList[i][1] = point0[1];
    } else{
      pointList[i][0] = pointList[i - 1][0] + cosTriangle;
      pointList[i][1] = pointList[i - 1][1] + sinTriangle;
      
    }
  }
  
  //double m = (deltaY)*(deltaX);
  //equalize(&pointList[0][0],result);

  /*
   * Gera os comandos em uma string
   */
  double theta[3];

  for (int i = 0; i < 3; i++) {
    theta[i] = 0;
  }

  for(int i = 0; i < discreteDistance; i++){

    inverseKinematic(pointList[i][0],pointList[i][1],0,&theta[0]);
    
    for(int j = 0; j < 3; j++){
    movs = addMotor(j,theta[j], movs);
    }
  
  }
}

//=====================================================================================================================================================================
//        Writing
//=====================================================================================================================================================================



/*
 * 
 *                                Letras:
 *  
 *  
 *                    . . .           0 1 2     --
 *                    . . .   ->      3 4 5    | 
 *                    . . .           6 7 8     --
 *                    
 *                   
 *                   
 */

String getCommandsToWriteWord(String wordToWrite){

  String movs = "";
  int numberOfLetters = 0;
/*
 * Acho que vai cortar mais uma letra por causa do < no for abaixo
 */
  if(wordToWrite.length() < MAXIMUM_LETTERS_TO_WRITE){
    numberOfLetters = wordToWrite.length();
  } else {
    numberOfLetters = MAXIMUM_LETTERS_TO_WRITE;
  }


  for(int i = 0; i < numberOfLetters; i++){
    char letter = wordToWrite[i];
    int letterOrder = letter - 'a';
    String wordCommands = words[letterOrder];

    double angle_bias = GRAUS_TO_RADIANS*(MAXIMUM_ANGULATION - ANGLE_BETWEEN_LETTERS*(i + NUMBER_OF_NO_LETTERS_AT_BEGINING_TO_WRITE)); // -90 para corrigir a diferenca de orientacao do robo com a matematica
    String pos = getMovsFromPositions(wordCommands, angle_bias);
    movs.concat(pos);
    //come back to inicial position
    for(int j = 0; j < 3; j++){
        movs = addMotor(j,SERVO_INIT_POS, movs);
    }
  }
  
  return movs;
}


String getMovsFromPositions(String positions, double angle_bias){

  String movs = "";

  double locs[(LENGTH_LETTER_MATRIX_TO_WRITE*DEEPTH_LETTER_MATRIX_TO_WRITE)][2];
  
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
  
        double radius = (TOP_RADIUS_TO_WRITE - DISTANCE_BETWEEN_DOTS_TO_WRITE*i);
        double angle = (DISTANCE_BETWEEN_DOTS_TO_WRITE/radius);
              
        //parametric equation of circle
        locs[3*i+j][0] = radius * cos(-1*angle*j + angle_bias);
        locs[3*i+j][1] = radius * sin(-1*angle*j + angle_bias);
      }
  }

  String inString = "";
  
  for (int i = 0; i < positions.length(); i++) {
    
    int inChar = positions[i];
    
    if (isDigit(inChar)) {
      inString += (char)inChar;

    } else if (inChar == '|') {

      int number = (int) inString.toInt();
      double x = locs[number][0];
      double y = locs[number][1];
      double z = POSITION_OF_Z_OF_GROUND;
      double theta[3];

      inverseKinematic(x,y,z,&theta[0]);
  
      for(int j = 0; j < 3; j++){
        movs = addMotor(j,theta[j], movs);
      }
     inString = "";
    }
  }
  return movs;
}

//============================================================
//        Print Methods
//============================================================

void userIntro(){
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
  Serial.println("exemplo: 0<-80|1<-90|2<+70|                                                                                         ");
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
