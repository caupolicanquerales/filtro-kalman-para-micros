/*
Libreria encargada de efectuar los calculos de operaciones sobre matrices del
orden de 2x2, 2x1, 1x2. 
Operaciones llevadas a cabo: multiplicacion P2x2 x C2x2
                             multiplicacion P2x2 x C2x1
                             multiplicacion P2x1 x C1x2
                             multiplicacion P1x2 x C2x1
                             suma           P2x2 + C2x2
                             suma           P2x1 + C2x1
*/


float r1_1;//primer elemento primera fila de la matriz 2x2
float r1_2;//segunda elemento primera fila de la matriz 2x2
float r2_1;//primer elemento segunda fila de la matriz 2x2
float r2_2;//segunda elemento segunda fila de la matriz 2x2


void metodoParaEfectuarMultipicacionDe2x2_2x2(float a1[],float a2[],float b1[],float b2[])
{
   r1_1=(a1[0]*b1[0])+(a1[1]*b2[0]);
   r1_2=(a1[0]*b1[1])+(a1[1]*b2[1]);
   r2_1=(a2[0]*b1[0])+(a2[1]*b2[0]);
   r2_2=(a2[0]*b1[1])+(a2[1]*b2[1]);
}

void metodoParaEfectuarMultiplicacion2x2_2x1(float a1[],float a2[],float b1[])
{
   r1_1=(a1[0]*b1[0])+(a1[1]*b1[1]);
   r1_2=(a2[0]*b1[0])+(a2[1]*b1[1]);
}
void metodoParaEfectuarMultiplicacion1x2_2x1(float a1[],float b1[])
{
   r1_1=(a1[0]*b1[0])+(a1[1]*b1[1]);
}
void metodoParaEfecuarMultiplicacion2x1_1x2(float a1[],float b1[])
{
   r1_1=a1[0]*b1[0];
   r1_2=a1[0]*b1[1];
   r2_1=a1[1]*b1[0];
   r2_2=a1[1]*b1[1];
}


void metodoParaEfectuarMultiplicacion1x2_2x2(float c1[],float a1[],float a2[])
{
   r1_1=c1[0]*a1[0]+c1[1]*a2[0];
   r1_2=c1[0]*a1[1]+c1[1]*a2[1];
}
void metodoParaEfectuarSuma2x2_2x2(float a1[],float a2[],float b1[],float b2[])
{
   r1_1=a1[0]+b1[0];
   r1_2=a1[1]+b1[1];
   r2_1=a2[0]+b2[0];
   r2_2=a2[1]+b2[1];
}

void metodoParaEfectuarResta2x2_2x2(float a1[],float a2[],float b1[],float b2[])
{
   r1_1=a1[0]-b1[0];
   r1_2=a1[1]-b1[1];
   r2_1=a2[0]-b2[0];
   r2_2=a2[1]-b2[1];
}

void metodoParaRestar2x1_2x1(float a1[],float b1[])
{
   r1_1=a1[0]-b1[0];
   r1_2=a1[1]-b1[1];
}

void metodoParaSumar2x1_2x1(float a1[],float b1[])
{
   r1_1=a1[0]+b1[0];
   r1_2=a1[1]+b1[1];
}

void metodoParaEfectuaMatrizTraspuesta2x2(float a1[],float a2[])
{  
   r1_1=a1[0];
   r1_2=a2[0];
   r2_1=a1[1];
   r2_2=a2[1];
}
void metodoParaMultiplicarMatriz2x1_numero(float a1[],float numero)
{
   r1_1=a1[0]*numero;
   r1_2=a1[1]*numero;
}

void metodoParaMultiplicarMatriz2x2_numero(float a1[],float a2[],float numero)
{
   r1_1= a1[0]*numero;
   r1_2= a1[1]*numero;
   r2_1= a2[0]*numero;
   r2_2= a2[1]*numero;
}

float metodoParaRegresarelemento1_1()
{
   return r1_1;
}
float metodoParaRegresarelemento1_2()
{
   return r1_2;
}
float metodoParaRegresarelemento2_1()
{
   return r2_1;
}
float metodoParaRegresarelemento2_2()
{
   return r2_2;
}
