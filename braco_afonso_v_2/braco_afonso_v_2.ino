
#include <Servo.h>

//============================================================
//        Declaring variables
//============================================================


//Arm sizes ans angles
double A_Zero = 11;
double A_Um = 16;

static const double theta0 = 0;
static const double theta1 = 0;
static const double theta2 = 90*2*M_PI/360;


int enable_training_pin = 3;

String inString = "";  

//Servo base_z;  
//Servo base_y;
//Servo braco;

static int BASE_Z = 0;
static int BASE_Y = 1;
static int BRACO_Y = 2;
static int SERVO_ID_BIAS = 4;
static int SERVO_POS_BIAS = 90;


Servo servos[3];

int servos_pos[3];
static int SERVO_INIT_POS = 90;

int pos_base_y = 90;
int pos_base_z = 90;
int pos_braco = 90;

//============================================================
//        Main Methods
//============================================================


void setup() { // Open serial communications and wait for port to open:
  initSerial();
  userIntro();
  initServos();

  double mat[4][4];
  double resultado[4][4];
  identity(&mat[0][0]);
                                             
  cinematicaDireta(theta0, theta1, theta2, A_Zero, A_Um, &mat[0][0], &resultado[0][0]);
  imprimir(resultado);

}

void loop() {
  //receiveData();
  
if(digitalRead(enable_training_pin) == HIGH){
  executeMovs(traningMode());
}

  }

//============================================================
//        Second line Methods
//============================================================


void operationMode(){
  int enable_traning = digitalRead(enable_training_pin);
  if(enable_traning == HIGH){
    //traninig mode activate
  } else {
    // traning mode deactivate
  }
}

void receiveData(){
  int motor_number = 0;
  while (Serial.available() > 0) {
    identifyFunctionAndActions(motor_number);
  }
}

void identifyFunctionAndActions(int qnt){
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      inString += (char)inChar;
      
    } else if (inChar == '|') {
    
      savePos(qnt,(int) inString.toInt());
      inString = "";
      qnt++;
    
    } else if (inChar == '\n') {
      inString = "";
      qnt = 0;
      Serial.println("finalizou");
    }
    if(qnt == 3){
      moveServos();//pos_base_y,  pos_base_z,  pos_braco);
    }
}

//Em contstrucao
void executeMovs(String movs){
  int servo_id = -1;
  for(int i = 0; i < movs.length(); i++){
      int inChar = movs[i];
      if (isDigit(inChar)) {
        inString += (char)inChar;
        
      } else if (inChar == '<') {
        // identify motor
        servo_id  = inString.toInt();
        Serial.println("Servo_id");
        Serial.println(servo_id);
        inString = "";
        
      } else if (inChar == '|') {
      
      //change here
        servos[servo_id].write((int) inString.toInt());
        inString = "";
        delay(100);
        
      
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
  if(enable == true){
    Serial.println("Traning mode enable");
  }
  String movs;
  int potpos;   
  int last_mov[3];
  int potpin = 4;
  
      while(enable){
      for(int i = potpin; i < potpin + 3; i++){
    
        potpos = map(analogRead(i), 0, 1023, 0, 180);     
    
        if(!areThisEqual(potpos,last_mov[i - potpin])){
          int servo_id = i - potpin;
          movs = addMotor(servo_id, potpos, movs);
          Serial.println("mov detected: ");
          Serial.println(potpos);
          last_mov[servo_id] = potpos;
          servos[servo_id].write(potpos);
          delay(25);
        } 
              

      }
      enable = (boolean)digitalRead(3);
      if(enable == false){
        Serial.println("ending...");
        Serial.print("movs: ");
        Serial.println(movs);
      }
    }
  return movs;
}

String addMotor(int servo_id, int potpos, String mov){
    
    mov.concat(String(servo_id));
    mov.concat("<");
    mov.concat(String(potpos));
    mov.concat("|");
   
    return mov;
}

//============================================================
//        Setup Methods
//============================================================

void initSerial(){
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void userIntro(){
  Serial.println("");
  Serial.println("Braco robótico");
  Serial.println("Para o controle direto dos motores, estreva: pos_base_y|pos_base_z|pos_braco|");
  Serial.println("exemplo: 80|90|100|");
}

void initServos(){

  for( int servo_id = BASE_Z; servo_id < BRACO_Y + 1; servo_id++){
    servos[servo_id].attach(SERVO_ID_BIAS + servo_id);
    servos_pos[servo_id] = SERVO_INIT_POS;
  }

}

//============================================================
//         Servo Moviment Control and methods
//============================================================


void moveServos(){//pos_base_y, int pos_base_z, int pos_braco){
  
   for( int servo_id = BASE_Z; servo_id < BRACO_Y + 1; servo_id++){
 
    servos[servo_id].write(servos_pos[servo_id]);
  }
}  

void savePos(int servo_id, int pos){
  servos_pos[servo_id] = pos;
}


//============================================================
//         Utils
//============================================================

//with filter
boolean areThisEqual(int a, int b){
  
  if(b < a - 8 || b > a + 8){
    return false;
  } else return true;
}

//============================================================
//         Kinematics
//============================================================

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


void cinematicaInversa(double x,double y,double z){

  static const int A_Zero = 11;
  static const int A_Um = 16;

  double w    = sqrt(pow(x,2)+pow(y,2));
  double gama = sqrt(pow(w,2)+pow(z,2));
  double alfa = acos((pow(A_Um,2) - pow(gama,2) - pow(A_Zero, 2))/(-2*gama*A_Zero));

  double teta0 = atan(y/x)*360/(2*M_PI);
  double teta1 = (acos(z/gama) - alfa)*360/(2*M_PI);
  double teta2 = (-1*acos((pow(gama,2) - pow(A_Zero,2) - pow(A_Um,2))/(-2*A_Zero*A_Um)) + M_PI/2)*360/(2*M_PI);
  
}


void cinematicaDireta(double theta0, double theta1, double theta2, double A_Zero, double A_Um, double *mat, double *result) 
{
    double mat1[4][4];
    rotateZ(mat,theta0, &mat1[0][0]);
    double mat2[4][4];
    rotateY(&mat1[0][0],theta1, &mat2[0][0]);
    double mat3[4][4];
    translateZ(&mat2[0][0],A_Zero,&mat3[0][0]);
    double mat4[4][4];
    rotateY(&mat3[0][0],theta2, &mat4[0][0]);
    double mat5[4][4];
    translateZ(&mat4[0][0],A_Um, &mat5[0][0]);
    double mat6[4][4];
    rotateY(&mat5[0][0],theta0, &mat6[0][0]);
    
    equalize(&mat6[0][0],result);
}




//Rotation methods
void rotateY(double *base, double theta, double *result){
  
  double rotY[4][4] = {{cos(theta), 0, sin(theta), 0},
                       {0, 1, 0, 0},
                       {-sin(theta), 0, cos(theta), 0},
                       {0 ,   0 ,    0 ,   1}};
                 
  mMatriz(base,&rotY[0][0], result);
  
}

void rotateZ(double *base, double theta, double *result){
  
    double rotZ[4][4] = {{cos(theta), -sin(theta), 0, 0},
                       {sin(theta), cos(theta), 0, 0},
                       {0, 0, 1, 0},
                       {0, 0, 0, 1}};
     
mMatriz(base,&rotZ[0][0], result);
}

void translateZ(double *base, double a, double *result){
  
      double transMatrix[4][4] =  {{1, 0, 0, 0},
                                   {0, 1, 0, 0},
                                   {0, 0, 1, a},
                                   {0, 0, 0, 1}};
                 
mMatriz(base,&transMatrix[0][0], result);
 
}

void mMatriz(double *a,double *b, double *result){
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<4;j++)
    {
      result[i*4+j]=0;
      for(int k=0;k<4;k++)
        result[i*4+j]=result[i*4+j]+a[i*4+k]*b[k*4+j];
    }
  }
}

void identityMatrix(double *mat){
  for(int t = 0; t < 4; t++){
    mat[t*4+t] = 1;
  }
}

void equalize (double *p, double *q)
{
   q = p;
}

void imprimir(double m[4][4]){
  for (int i=0;i<=3;i++){
    Serial.print("[ ");
    for (int j=0;j<=3;j++){
     Serial.print(m[i][j]);
    if (j<3) Serial.print(" , ");
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
                 mat[row*num+col] = 1; 
            else
                mat[row*num+col] = 0;
        }  
    }  
} 


  
