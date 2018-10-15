class Kinematic
{
  public: 
  void teste(){
    Serial.println("deu certo");
  }
  
};

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  Kinematic kinematic;
  kinematic.teste();

}
