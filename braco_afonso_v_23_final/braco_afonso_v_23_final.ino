
#include <Servo.h>


  /*
   * Davi Guanabara - Robótica Industrial Unifor
   * 
   * Não sei pq porra de motivo a funcao forwardKinematic(), se usado abaixo, da pau pra sempre!!! 
   * Problemas foram identificados na funcao isAnglesForbbiden, generateLine, getCommandsToWriteWord
   * 
   * Replicar Bug: 
   *      1) boot do robo com o pino 3 alimentado com VCC (5v)
   *      2) alimentar o pino 3 com GND
   *      3) escolher a opção treinamento (b)
   *      4) alimentar o pino 3 com VCC (5v)
   *      5) escolher a opção treinamento (b)
   *      6) o programa irá travar no função treinamento  
   *      
   *      
   */

//============================================================
//        Declaring variables
//============================================================

const PROGMEM String words[25] =   {"0|1|2|5|8|5|4|3|6|", //A
                                    "0|3|6|7|5|4|1|0|",
                                    "1|0|3|6|7|",
                                    "0|1|5|7|6|3|0|",
                                    "1|0|3|4|3|6|7|",
                                    "1|0|3|4|3|6|",
                                    "1|0|3|6|7|5|4|",
                                    "0|3|6|3|4|5|2|5|8|",
                                    "0|1|2|1|4|7|6|7|8|",
                                    "1|4|7|6|", // j
                                    "0|3|6|3|1|3|7",
                                    "0|3|6|7|",
                                    "6|3|0|7|2|5|8|",
                                    "7|4|1|8|5|2|",
                                    "0|3|6|7|4|1|0|", 
                                    "3|4|1|0|3|6|",
                                    "0|3|6|7|8|5|2|1|4|",
                                    "3|4|1|0|3|6|3|7|",
                                    "1|0|3|4|7|6|",
                                    "0|1|2|1|4|7|", 
                                    "0|3|6|7|4|1|",
                                    "0|7|2|",
                                    "0|4|8|4|2|4|6|",
                                    "0|4|1|4|7|",
                                    "0|1|2|4|6|7|8|"};   //Z


//Writing Configurations
static const int BASE_RADIUS_TO_WRITE = 10;
static const int LENGTH_LETTER_MATRIX_TO_WRITE = 3; 
static const int DEEPTH_LETTER_MATRIX_TO_WRITE = 3;
static const int DISTANCE_BETWEEN_DOTS_TO_WRITE = 1;
static const int NUMBER_OF_NO_LETTERS_AT_ENDING_TO_WRITE = 0;
static const int NUMBER_OF_NO_LETTERS_AT_BEGINING_TO_WRITE = 0;
static const int NUMBER_OF_NO_LETTERS = NUMBER_OF_NO_LETTERS_AT_BEGINING_TO_WRITE + NUMBER_OF_NO_LETTERS_AT_ENDING_TO_WRITE;
static const int TOP_RADIUS_TO_WRITE = DEEPTH_LETTER_MATRIX_TO_WRITE + BASE_RADIUS_TO_WRITE;
static const int MAXIMUM_LETTERS_TO_WRITE = (int) ((M_PI*BASE_RADIUS_TO_WRITE/(DISTANCE_BETWEEN_DOTS_TO_WRITE*(LENGTH_LETTER_MATRIX_TO_WRITE))) - NUMBER_OF_NO_LETTERS); // 1 POR PREUCAUCAO  9;

static const int MAXIMUM_ANGULATION = 90;
static const int MAXIMUM_ABSOLUTE_ANGULATION = 180;
static const int ANGLE_BETWEEN_LETTERS = MAXIMUM_ABSOLUTE_ANGULATION/(MAXIMUM_LETTERS_TO_WRITE + NUMBER_OF_NO_LETTERS);

// conversions
static const double GRAUS_TO_RADIANS = (2 * M_PI / 360);
static const double RADIANS_TO_GRAUS = 360 / (2 * M_PI);

// Enable pins
static const int RECEIVER_DELAY = 1;
static const int ENABLE_TRAINING_PIN = 3;

//Arm sizes
static const double A_Um = 16;
static const double A_Zero = 9.5;
static const double A_MenosUm = 5;
static const double A_Menos2 = 3;
static const double A_Elbow = 8;

//Servo Configurations
static const int BASE_Z = 0;
static const int BASE_Y = 1;
static const int BRACO_Y = 2;

static const int SERVO_DELAY = 10;
static const int SERVO_ID_BIAS = 4;
static const int SERVO_INIT_POS = 0;
static const int SERVO_POS_BIAS = 90;

static const double Theta0_BIAS = 0;
static const double Theta1_BIAS = 0;
static const double Theta2_BIAS = 90;

static const int FILTER_FOR_POTENTIOMETER = 5;

static const int MINIMUM_DISTANCE_ALLOWED_TO_GROUND = 0;
static const int MINIMUM_DISTANCE_ALLOWED_TO_ORIGIN = 7;

//Intantiate servo
Servo servos[3];
double currentServoAngles[3];



//============================================================
//        Main Methods
//============================================================


void setup() { // Open serial communications and wait for port to open:
  initSerial();
  setupPins();
  setupPinCable();
  Serial.println("Booting Arm...");
  userIntro();
  initServos();

  kinematicsDemo();
  Serial.println("Digite a para treinar ou c para enviar um comando");
}

void loop() {

  //int enable_traning = digitalRead(ENABLE_TRAINING_PIN);
  if(Serial.available()>0){

    char option = receiveData()[0];
    String movs = "";
    movs = getCommands(digitalRead(ENABLE_TRAINING_PIN),option);
    if(movs.length()>0){
      Serial.println("movs: ");
      Serial.println(movs);
      executeMovs(movs);
    }
    Serial.println("Digite a para treinar ou b para enviar um comando");
  }
}



//============================================================
//        Second line Methods
//============================================================


String getCommands(int enable_traning, char command) {

  if(command == 'a'){
  if (enable_traning == HIGH) {
    Serial.println("Você digitou a");
    Serial.println("Modo de treinamento ativado");
    return traningMode();
    
  } else{
    Serial.println("porta 3 está inativa");
  }
  } else if(command == 'e'){
    /*
     * ainda em testes
     */
    Serial.println("Você digitou escrever");
    Serial.println("Esta funcionalidade ainda está em desenvolvimento.");
    Serial.println("Digite uma palavra para ser escrita: ");
    waitForUser();
    return getCommandsToWriteWord(receiveData());
   
    } else if(command = 'b'){

      Serial.println("Você digitou b");

      Serial.println("Escreva os comandos ");
      waitForUser();
      return receiveData();
    } else {
      Serial.println("Digite um comando valido");
      return "";
    }
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

void waitForUser(){
        while(Serial.available()<1){
        ;
      }
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

  delay(15);
  //0<-79|1<-19|2<-52|2<-43|1<-10|1<-1|0<-69|0<-60|0<-51|0<-41|0<-32|0<-23|0<-14|0<-5|0<+4|2<-34|0<+13|0<+22|2<-24|0<+31|2<-15|1<-10|1<-19|1<-28|2<-24|2<-33|2<-43|1<-37|1<-28|1<-19|1<-10|1<-1|1<+8|1<-1|0<+22|0<+13|0<+4|0<-5|0<-14|

  String movs;
  movs  = addMotor(0, currentServoAngles[0], movs);
  movs  = addMotor(1, currentServoAngles[1], movs);
  movs  = addMotor(2, currentServoAngles[2], movs);
  int potpos;
  int last_mov[3];
  int potpin = 4;
  
  while (enable) {
    for (int i = potpin; i < potpin + 3; i++) {

      potpos = map(analogRead(i), 0, 1023, -90, 90);
      if(i == potpin){
        potpos = -1*potpos;
      }

      if (!areThisEqual(potpos, last_mov[i - potpin])) {
        int servo_id = i - potpin;
        movs = addMotor(servo_id, potpos, movs);
        Serial.println("mov detected: ");
        Serial.println(potpos);
        moveServo(servo_id,potpos);
        last_mov[servo_id] = potpos;
        
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

void setupPins(){
  pinMode(ENABLE_TRAINING_PIN, INPUT);
}

void setupPinCable(){
    boolean enable = (boolean) digitalRead(ENABLE_TRAINING_PIN);
  if(enable == false){
  Serial.println("Alimente o pino digital 3 com 5v para iniciar a aplicação");
    while(!enable){
      enable = (boolean) digitalRead(ENABLE_TRAINING_PIN);
      delay(15);
    }
  while(Serial.available())
  Serial.read();
  }
}

void initSerial() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}


void kinematicsDemo() {
  double theta[3];

  Serial.println("");
  Serial.println(F("Kinematics Demo: "));

  for (int i = 0; i < 3; i++) {
    theta[i] = 0;
  }

  theta[0] = 40;

  printAngles(theta);

  double mat[4][4];
  double resultado[4][4];
  identity(&mat[0][0]);

  Serial.println("");
  Serial.println(F("Foward kinematic: "));

  forwardKinematic(theta, &mat[0][0], &resultado[0][0]);
  printMatrix(resultado);

  Serial.println("");
  Serial.println(F("Inverse kinematic: "));

  printPoint(resultado[0][3],resultado[1][3],resultado[2][3]);

  double beta[3];
  inverseKinematic(resultado[0][3],resultado[1][3],resultado[2][3], &beta[0]);
  
  printAngles(beta);
}

void initServos() {

  for ( int servo_id = BASE_Z; servo_id < BRACO_Y + 1; servo_id++) {
    servos[servo_id].attach(servo_id + SERVO_ID_BIAS);
    moveServo(servo_id, SERVO_INIT_POS);
  }

}


//============================================================
//         Servo Moviment Control and methods
//============================================================

void moveServo(int servo_id, int servo_pos) {

  if(!isAngleForbbiden(servo_id,servo_pos)){

    currentServoAngles[servo_id] = servo_pos;
    //printServoMoviment(servo_id, servo_pos, "Angulo: ");
    if(servo_id == 2){
      servos[servo_id].write(-1*servo_pos + SERVO_POS_BIAS);
    } else {
      servos[servo_id].write(servo_pos + SERVO_POS_BIAS);
    }
    //Serial.print(".");
    delay(SERVO_DELAY);
  } else {
    Serial.println("");
    printServoMoviment(servo_id, servo_pos, "Angulo proibido");
    //Serial.println("");
  }
}

//============================================================
//         Utils


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

  double w0    = sqrt(pow(x, 2) + pow(y, 2));
  double w    = w0 + A_Menos2;
  double gama = sqrt(pow(w, 2) + pow((z - A_MenosUm), 2));
  double alfa = acos((pow(A_Um, 2) - pow(gama, 2) - pow(A_Zero, 2)) / (-2 * gama * A_Zero));
  
  theta[0] = atan(y / x) * RADIANS_TO_GRAUS;
  theta[1] = (atan(w/(z-A_MenosUm)) - alfa) * RADIANS_TO_GRAUS;//(acos((z - A_MenosUm)/ gama) - alfa) * RADIANS_TO_GRAUS;
  theta[2] = (-1 * acos((pow(gama, 2) - pow(A_Zero, 2) - pow(A_Um, 2)) / (-2 * A_Zero * A_Um)) + M_PI / 2) * RADIANS_TO_GRAUS;
  //printAngles(theta);
}


void forwardKinematic(double theta[3], double *mat, double *result)
{
  theta[0] = (theta[0] + Theta0_BIAS) * GRAUS_TO_RADIANS;
  theta[1] = (theta[1] + Theta1_BIAS) * GRAUS_TO_RADIANS;
  theta[2] = (theta[2] + Theta2_BIAS) * GRAUS_TO_RADIANS;

  double mat1[4][4];
  rotateZ(mat, theta[0], &mat1[0][0]);
  double mat2[4][4];
  translateZ(&mat1[0][0], A_MenosUm, &mat2[0][0]);
  double matT[4][4];
  translateX(&mat2[0][0], -A_Menos2, &matT[0][0]);
  double mat3[4][4];
  rotateY(&matT[0][0], theta[1], &mat3[0][0]);
  double mat4[4][4];
  translateZ(&mat3[0][0], A_Zero, &mat4[0][0]);
  double mat5[4][4];
  rotateY(&mat4[0][0], theta[2], &mat5[0][0]);
  double mat6[4][4];
  translateZ(&mat5[0][0], A_Um, result);

  //equalize(&mat5[0][0], result);
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

void translateX(double *base, double translation, double *result) {

  double transMatrix[4][4] =  {{1, 0, 0, translation},
                               {0, 1, 0, 0},
                               {0, 0, 1, 0},
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


//============================================================
//        Print Methods
//============================================================

void userIntro(){
  Serial.println("");
  Serial.println(F("Davi Guanabara - Rodrigo Aguiar "));
  Serial.println(F("Braço robótico - perna do exapod"));
  Serial.println(F("                                                                                                                    "));
  Serial.println(F("                                                                                                                    "));
  Serial.println(F("                                                                                                                    "));
  Serial.println(F("                                                                                                                    "));
  Serial.println(F("                     a1 = 16 cm                                                                                     "));
  Serial.println(F("             (rotY) .__________                                                                                     "));
  Serial.println(F("                    |                                                                                               "));
  Serial.println(F("                    |                                                                                               "));
  Serial.println(F(" a0 = 11 cm         |                                                                                               "));
  Serial.println(F("                    . rotY                                                                                              "));
  Serial.println(F("                    | transZ(5cm)                                                                                   "));
  Serial.println(F("                    . rotZ                                                                                    "));
  Serial.println(F("                                                                                                                    "));
  Serial.println(F("                                                                                                                    "));
  Serial.println(F("M02  =  RotationZ(theta0 + 0)*TranslationZ(5)*RotationY(theta1 + 0)*TranslationZ(A0)*RotationY(theta2 + 90)*TranslationZ(A1)        "));
  Serial.println(F("                                                                                                                    "));
  Serial.println(F("Para o controle direto dos motores, escreva: id do motor<posição do motor|                                          "));
  Serial.println(F("exemplo: 0<-80|1<-90|2<+70|                                                                                         "));
  Serial.println(F(""));
  Serial.flush();
}


void printPoint(double x, double y, double z){
  Serial.println(F("x: "));
  Serial.print(x);
  Serial.println(F(" y: "));
  Serial.print(y);
  Serial.println(F(" z: "));
  Serial.println(z);
  Serial.println(F(""));
  Serial.flush();
}

void printServoMoviment(int servo_id, int angle, String msg ){
  Serial.print(F("Servo_id: "));
  Serial.print(servo_id);
  Serial.print(F(" Angulo do motor: "));
  Serial.print(angle);
  Serial.print(F(" msg:"));
  Serial.println(msg);
  Serial.flush();
}

void printAngles(double theta[3]){
  for (int i = 0; i < 3; i++) {
    Serial.print(F("Theta"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(theta[i]);
  }
  Serial.flush();
}

void printMatrix(double m[4][4]) {
  Serial.println("");
  for (int i = 0; i <= 3; i++) {
    Serial.print("[ ");
    for (int j = 0; j <= 3; j++) {
      Serial.print(m[i][j]);
      if (j < 3) Serial.print(" , ");
      else Serial.println(" ]");
    }
  }
  Serial.println("");
}


//=====================================================================================================================================================================
//        Moviments Blocked
//=====================================================================================================================================================================


boolean isGround(double x, double y, double z){
  if(z < MINIMUM_DISTANCE_ALLOWED_TO_GROUND){ 
    Serial.println("vai enconstar no chão");
    return true;
  }
  else return false;
}

boolean isBase(double x, double y, double z){
  if(sqrt(pow(x,2) + pow(y,2) + pow(z,2)) < MINIMUM_DISTANCE_ALLOWED_TO_ORIGIN){ 
    Serial.println("vai enconstar na base");
    return true;
    }
  else return false;
}

boolean isPositionForbbiden(double mat[4][4], double mat1[4][4]){

  double x  = mat[0][3];
  double y  = mat[1][3];
  double z  = mat[2][3];
  double x1 = mat1[0][3];
  double y1 = mat1[1][3];
  double z1 = mat1[2][3];

  //printPoint(x,y,z);

  if(isBase(x,y,z) || isGround(x,y,z) || isGround(x1,y1,z1)) return true;
  else return false;


  
//  double mat1[4][4];
//  //cotovelo
//  
//  x = mat1[0][3];
//  y = mat1[1][3];
//  z = mat1[2][3];
//  
//  printPoint(x,y,z);
//  
//  if(isBase(x,y,z) || isGround(x,y,z)) return true;
  
  return false;
  
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
  translateZ(&mat1[0][0], A_MenosUm, &mat2[0][0]);
  double mat3[4][4];
  rotateY(&mat2[0][0], theta[1], &mat3[0][0]);
  double mat4[4][4];
  translateZ(&mat3[0][0], A_Zero, &mat4[0][0]);
  double mat5[4][4];
  rotateY(&mat4[0][0], theta[2], &mat5[0][0]);
  double mat6[4][4];
  translateZ(&mat5[0][0], A_Um, &mat6[0][0]);
  double mat7[4][4];
  translateZ(&mat5[0][0], - A_Elbow, &mat7[0][0]);

  printMatrix(mat6);

  return isPositionForbbiden(mat6, mat7);
}

boolean isAngleForbbiden(int servo, double angle){

  double angles[3];
  

  angles[0] = currentServoAngles[0];
  angles[1] = currentServoAngles[1];
  angles[2] = currentServoAngles[2];
  angles[servo] = angle;


  return isAnglesForbbiden(angles);
}


//=====================================================================================================================================================================
//        Geometry
//=====================================================================================================================================================================



String generateLine(double point0[2],double point1[2], String movs){ //, double *result){

  double deltaY = point1[1] - point0[1];
  double deltaX = point1[0] - point0[0];

  double distance = sqrt(pow(deltaY,2) + pow(deltaX,2));

  double cosTriangle = deltaX/distance;
  double sinTriangle = deltaY/distance;
  
  int discreteDistance = (int) distance;

  double lineStep = distance/discreteDistance;

  double pointTemp[2];
  double pointList[discreteDistance][2];
  
  Serial.print("discreteDistance ");
  Serial.print(discreteDistance);
  //Serial.println("");

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
  return movs;
}

//=====================================================================================================================================================================
//        Writing
//=====================================================================================================================================================================

/*
 * x = cx + r * cos(a)
 * y = cy + r * sin(a)
 * Where r is the RADIUS, cx,cy the origin, and a the angle.
 */


/*
 * 
 *                                Letras:
 *  
 *  
 *                    . . .        31 32 33   0 1 2
 *                    . . .   ->   21 22 23   3 4 5
 *                    . . .        11 12 13   6 7 8 
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
   Serial.println(MAXIMUM_LETTERS_TO_WRITE);
  Serial.println("Maximum letters: ");
  if(wordToWrite.length() < MAXIMUM_LETTERS_TO_WRITE){
    numberOfLetters = wordToWrite.length();
  } else {
    numberOfLetters = MAXIMUM_LETTERS_TO_WRITE;
  }
  Serial.print(F("Palavra identificada: "));
  Serial.print(wordToWrite);
  Serial.println(F("Os comandos serã gerados "));
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
  Serial.println(movs);
  Serial.flush();
  Serial.println(F("Os comandos foram gerados com sucesso "));
  return movs;
}


String getMovsFromPositions(String positions, double angle_bias){


  String movs = "";

  double locs[(LENGTH_LETTER_MATRIX_TO_WRITE*DEEPTH_LETTER_MATRIX_TO_WRITE)][2];
  
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
  
        double radius = (TOP_RADIUS_TO_WRITE - i*DISTANCE_BETWEEN_DOTS_TO_WRITE);
        double angle = (DISTANCE_BETWEEN_DOTS_TO_WRITE/radius);
              Serial.println(angle);
        //parametric equation of circle
        locs[3*i+j][0] = radius * cos(-1*angle*j + angle_bias);
        locs[3*i+j][1] = radius * sin(-1*angle*j + angle_bias);
      }
  }

  String inString = "";

  
  int lastNumber = -1;
  for (int i = 0; i < positions.length(); i++) {
    
    int inChar = positions[i];
    
    if (isDigit(inChar)) {
      inString += (char)inChar;

    } else if (inChar == '|') {

      int number = (int) inString.toInt();
      double x = locs[number][0];
      double y = locs[number][1];
      double z = 0;
      
      double theta[3];

      if(lastNumber != -1){
        Serial.println("generateLine");
        movs = generateLine(locs[lastNumber],locs[number], movs);
        Serial.println(movs);
      } else {
  
        inverseKinematic(x,y,z,&theta[0]);
        for(int j = 0; j < 3; j++){
          movs = addMotor(j,theta[j], movs);
        }
      }

     lastNumber = number;
     inString = "";
    }
  }
  return movs;
}
