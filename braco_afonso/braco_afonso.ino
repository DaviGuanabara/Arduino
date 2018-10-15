
#include <Servo.h>

//============================================================
//        Declaring variables
//============================================================


int enable_training_pin = 3;

String inString = "";  

Servo base_z;  
Servo base_y;
Servo braco;

static int BASE_Z = 0;
static int BASE_Y = 1;
static int BRACO_Y = 2;
static int SERVO_ID_BIAS = 4;
static int SERVO_POS_BIAS = 90;


Servo servo[3];

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

}

void loop() {
  //receiveData();
  
if(digitalRead(enable_training_pin) == HIGH){
  executeMovs(traningMode());
}
  delay(10000);
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
  for(int i = 0; i < movs.length(); i++){
      int inChar = movs[i];
      if (isDigit(inChar)) {
        inString += (char)inChar;
        
      } else if (inChar == '<') {
        // identify motor
      } else if (inChar == '|') {
      
      //change here
        base_z.write((int) inString.toInt());
        delay(100);
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
          movs = addMotor(i, potpos, movs);
          Serial.println("mov detected: ");
          Serial.println(potpos);
          last_mov[i - potpin] = potpos;
          //change here:
          base_z.write(potpos);
          delay(100);
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

String addMotor(int motor, int val, String mov){
    
    mov.concat(String(1000 + motor));
    mov.concat("<");
    mov.concat(String(val));
    mov.concat("|");
   
    return mov;
}

//int getPotInt(int potpin){
//   return map(analogRead(potpin), 0, 1023, 0, 180);   
//}
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
    servo[servo_id].attach(SERVO_ID_BIAS + servo_id);
    servos_pos[servo_id] = SERVO_INIT_POS;
  }

//  base_y.attach(4);  
//  base_z.attach(5);
//  braco.attach(6);
}

//============================================================
//         Servo Moviment Control and methods
//============================================================


void moveServos(){//pos_base_y, int pos_base_z, int pos_braco){
  
   for( int servo_id = BASE_Z; servo_id < BRACO_Y + 1; servo_id++){
 
    servo[servo_id].write(servos_pos[servo_id]);
  }

  
//  base_z.write(pos_base_y);
//  base_y.write(pos_base_z);
//  braco.write(pos_braco);
}  

void savePos(int servo_id, int pos){
  servos_pos[servo_id] = pos;
  
//        if(motor == 0){
//            pos_base_y = pos;
//            Serial.print("pos_base_y: ");
//            Serial.println(pos_base_y);
//    }
//        if(motor == 1){
//            pos_base_z = pos;
//            Serial.print("pos_base_z: ");
//            Serial.println(pos_base_z);
//    }
//        if(motor == 2){
//            pos_braco = pos;
//            Serial.print("pos_braco: ");
//            Serial.println(pos_braco);
//    }  
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



  
