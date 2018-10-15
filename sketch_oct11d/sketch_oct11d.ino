
/*
 * https://www.ime.usp.br/~pf/algoritmos/aulas/pont.html
 * https://www.ime.usp.br/~hitoshi/introducao/23-matrizes_ponteiros.pdf
 * 
 * 
 * 
 */

double A_Zero = 11;
double A_Um = 16;

static const double theta0 = 0;
static const double theta1 = 0;
static const double theta2 = 90*2*M_PI/360;

void setup() {
  // put your setup code here, to run once:



  Serial.begin(9600);

                       
  double mat[4][4];
  double resultado[4][4];
  identity(&mat[0][0]);
                                             
  cinematicaDireta(theta0, theta1, theta2, A_Zero, A_Um, &mat[0][0], &resultado[0][0]);
  imprimir(resultado);
                 
}



void loop() {
  // put your main code here, to run repeatedly:
   

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
