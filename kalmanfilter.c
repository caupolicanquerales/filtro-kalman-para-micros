
#include "D:\Users\Capo\Desktop\microcontroller\programas en pic\operacionesMatrices.c"

#include <math.h>
#define PI 3.14159265

void metodoParaValor00(int8);
void metodoParaValor01(int8);
void metodoParaValor10(int8);
void metodoParaValor11(int8);
void metodoParaMatriz_X00(int8);
void metodoParaMatriz_X01(int8);
void metodoParaMatriz_X10(int8);
void metodoParaMatriz_X11(int8);
void metodoParaCargarResultado2x2();

/*
Libreria encargada de ejecutar y coordinar la secuencia de pasos requeridos para
ejecutar el procedimiento del FILTRO de KALMAN

   P(k/k) matriz de probabilidad----- matriz P(2x2) de filas P1(1x2) y P2(1x2)
   P(k+1/k) matriz de probabilidad----- matriz P_adelante(2x2) de filas P_adelante1 (1x2) y P_adelante2(1x2)
   A matriz modelo del encoder----- matriz A (2x2) de filas A1(1x2) y A2(1x2)
   Q matriz ----- matriz Q (2x2) de filas Q1(2x1) y Q2(2x1)
   P(k+1/k+1) matriz de probabilidad corregida---- matriz P_corregida(2x2) de filas P_corregida1 (1x2) y P_corregida2 (1x2)
   K ganancia---- matriz ganancia_K (2x1) de fila ganancia_K (1x2)
*/ 

/******************************************************************************
   Matriz P inicial para la interación del filtro de Kalman
   
   P=[T^((2n-2i+1)/(2n)) 0;0 T^((2n-2i+1)/(2n))]
   n----> orden de la cantidad de medidas (velocidad y orientacion)
   T----> tiempo de muestreo
   i----> posicion en la matriz
   
   Matriz Q ----> Q= E[w(kt)*wt(kt)]
                  Q= q*[1/3*T^3  1/2*T^2; 1/2*T^2  T]
          q ----> factor de entonacion
   
   Matriz b ----> b=[ 1/2*T^2; T]
   
   Matriz A ----> A=[1 T;0 1]  modela el encoder para calcular velocidad y orientación
*******************************************************************************/

/****************************************************************************
   Secuencia de ejecucion de los metodos:
   (estructura de secuencia a cumplir si se desea implementar el filtro de kalman)
   
   
   inicial: (primera iteracion)
      metodoParaCalcularMatrices_A_b_Q_P(float tiempo,float q)
      metodoParaCalcularVariables(int bajo,int alto,int16 pulsoEncoder,float tiempo)
      metodoParaEfectuarCalculo_X_futuro()
      metodoParaEfectuarCalculo_P_futuro()
      metodoParaCalcularGananciaKalman(float R)
      metodoParaCalcularCorreccionProbabilidad()
      
      metodoParaCalcularVariables(int bajo,int alto,int16 pulsoEncoder,float tiempo) segunda medicion.
      metodoParaCorregirLaMatriz_X_predicha() 

   Parte Iterativa: (ejecucion del metodo)
      metodoParaEfectuarCalculo_X_futuro()
      metodoParaEfectuarCalculo_P_futuro()
      metodoParaCalcularGananciaKalman(float R)
      metodoParaCalcularCorreccionProbabilidad()
      metodoParaCalcularVariables(int bajo,int alto,int16 pulsoEncoder,float tiempo)
      metodoParaCorregirLaMatriz_X_predicha()
*******************************************************************************/


float A1[2]; //primera fila de matriz A
float A2[2]; //segunda fila de matriz B
float Q1[2]; //primera fila de matriz Q
float Q2[2]; //segunda fila de matriz Q
float b[2];//matriz que modela la acelaracion.
float c1[2]={1.0,0.0};
float c2[2]={0.0,1.0};
float resultado1[2];
float resultado2[2];
float transpuesta1[2];
float transpuesta2[2];
float Matriz_P1[2]={0.0,0.0}; //primera fila de matriz P(k+1/k+1)
float Matriz_P2[2]={0.0,0.0}; //segunda fila de matriz P(k+1/k+1)
float ganancia_K1[2]={0.0,0.0};
float ganancia_K2[2]={0.0,0.0};
float matriz_X[2];// matriz de las medidas "angulo" y "velocidad angular"
float matrizMedida_X[2];// matriz de medidas iniciales "angulo" y "velocidad angular"
//float valor,factor;
//float ak=2.0;//aceleracion angular (variable de control)
int16 pulso=0;
int16 pulso2;
int16 valor1=0;
int16 valor2=0;
//int16 valor3,valor4,valor5,valor6;
int i,primero=0;
int b1,al1,b2,al2;


/****************************************************************************
   Metodo para calcular las variables "Velocidad angular", "Orientacion" y
   "Aceleración angular", dado los pulsos medidos por el encoder.
   
   velocidad angular ----> w=(2*PI*delta_Pulsos)/(Pulsos*Tsc)
******************************************************************************/
void metodoParaCalcularVariables(int bajo,int alto,int16 pulsoEncoder,float tiempo)
{
   pulso= (int16)(alto)*256+(int16)bajo;
  
   if(pulso>pulsoEncoder)
   {
      pulso2=pulso-pulsoEncoder;
   }else{
      pulso2=pulso;
   }
 //Estructura de decision para determinar las variables medidas
   if(primero==0) 
   {
   //Estructura para la primera captura de valores que iniciara el Filtro de Kalman
      primero=1;
      valor1=100*(2*PI*pulso2)/pulsoEncoder;//posicion angular
      valor2=100*(2*PI*pulso)/(pulsoEncoder*tiempo);//velocidad angular
   }else{
   //Estructura empleada para posteriores capturas de medidas ya inicializado el Filtro de Kalman
      matrizMedida_X[0]=(2*PI*pulso2)/pulsoEncoder;//posicion angular
      matrizMedida_X[1]=(2*PI*pulso)/(pulsoEncoder*tiempo);//velocidad angular
   }
}

/****************************************************************************
   Metodo para calcular Matriz A, b, Q y P. Matriz que dependen del tiempo de
   muestreo. P(k/k) inicial. Al momento de iniciar el
   ciclo iterativo de Kalman.   
******************************************************************************/
void metodoParaCalcularMatrices_A_b_Q_P(float tiempo,float q)
{
   A1[0]=1;
   A1[1]=tiempo;
   A2[0]=0;
   A2[1]=1;
   
   Q1[0]=q/3*pow(tiempo,3);
   Q1[1]=q/2*pow(tiempo,2);
   Q2[0]=q/2*pow(tiempo,2);
   Q2[1]=tiempo*q;
   
   b[0]=pow(tiempo,2)/2;
   b[1]=tiempo;
   
 ///////////////////////////////////////////////////////////////////////////
 /* valor inicial para la matriz P(k-1), solo se ejecutara al inicio del programa*/
   //valor=(4-1)/4;
   //valor=(4-3)/4;
   valor1=100*pow(tiempo,0.75);
   valor2=100*pow(tiempo,0.25);
////////////////////////////////////////////////////////////////////////////   
}

/*****************************************************************************
   metodo para calcular x(k+1/k)
                       x(k+1/k)= A(k)*x(k/k) + b(k)*u(k)
                       u(k)----> es el valor de la aceleracion medida (ak)
******************************************************************************/
void metodoParaEfectuarCalculo_X_futuro(float ak)
{
/* Multiplicación A(k)*x(k/k)= R */
   metodoParaEfectuarMultiplicacion2x2_2x1(A1,A2,matriz_X);
   resultado1[0]= metodoParaRegresarelemento1_1();
   resultado1[1]= metodoParaRegresarelemento1_2();
   
/* Multiplicación b(k)*u(k)= R2 */
   metodoParaMultiplicarMatriz2x1_numero(b,ak);
   resultado2[0]= metodoParaRegresarelemento1_1();
   resultado2[1]= metodoParaRegresarelemento1_2();
   
/* Suma R + R2 = x(k+1/k)*/
   metodoParaSumar2x1_2x1(resultado1,resultado2);
   matriz_X[0]= metodoParaRegresarelemento1_1();
   matriz_X[1]= metodoParaRegresarelemento1_2();

}

/*****************************************************************************
   metodo para calcular P(k+1/k)
                       P(k+1/k)=A(k)*P(k/k)*At(k) + Q
*******************************************************************************/
 
void metodoParaEfectuarCalculo_P_futuro(float tiempo)
{
   
/* Multiplicacion A(k)*P(k/k)=R */
   metodoParaEfectuarMultipicacionDe2x2_2x2(A1,A2,matriz_P1,matriz_P2);
   metodoParaCargarResultado2x2();  
     
  /* Transpuesta de A*/ 
   transpuesta1[0]=1;
   transpuesta1[1]=0;
   transpuesta2[0]=tiempo;
   transpuesta2[1]=1;
   
/* Multiplicacion R*At(k) =R2 */
   metodoParaEfectuarMultipicacionDe2x2_2x2(resultado1,resultado2,transpuesta1,transpuesta2);
   metodoParaCargarResultado2x2();
   
      
/* Suma de R2 +Q = P(k+1/k) (valores por encima y debajo de diagonal principal igual a cero)*/   
   metodoParaEfectuarSuma2x2_2x2(resultado1,resultado2,Q1,Q2);
   matriz_P1[0]= metodoParaRegresarelemento1_1();
   matriz_P2[1]= metodoParaRegresarelemento2_2();
   
}

/*********************************************************************
* Metodo para el calculo de la ganancia K(t+1)
       K(k+1)=P(k+1/k)*Ct(k+1)*[C(k+1)*P(k+1/k)*Ct(k+1) + R]^-1
       
       los valores de P(k+1/K) son los valores de P_adelante1 y P_adelante2
**********************************************************************/

void metodoParaCalcularGananciaKalman(float R1[],float R2[])
{

/* multiplicacion de C(k+1)*P(k+1/k)= R*/
   metodoParaEfectuarMultipicacionDe2x2_2x2(C1,C2,matriz_P1,matriz_P2);
   metodoParaCargarResultado2x2();
   
 
 /* multiplicacion de R*Ct(k+1) = R2 */
   metodoParaEfectuarMultipicacionDe2x2_2x2(resultado1,resultado2,C1,C2);
   metodoParaCargarResultado2x2();
   
 /* en este paso falta sumar "valor" al error de medida R requerido por Kalman para
 despues invertir el valor que sera multiplicado por R3. 
    suma R2 + R(error)= R3 */  
   metodoParaEfectuarSuma2x2_2x2(resultado1,resultado2,R1,R2);
   metodoParaCargarResultado2x2();
   
 /* multiplicacion de P(k+1/k)*Ct(k+1) = R4  */
   metodoParaEfectuarMultipicacionDe2x2_2x2(matriz_P1,matriz_P2,C1,C2);
   transpuesta1[0]= metodoParaRegresarelemento1_1();
   transpuesta1[1]= metodoParaRegresarelemento2_2();   
     
 /* calculo de ganancia K(k+1)= R3/R4 */
   ganancia_K1[0]=transpuesta1[0]/resultado1[0];
   ganancia_K2[1]=transpuesta1[1]/resultado2[1];
   
}

/**************************************************************************
   Metodo para calcular la Probabilidad P(k+1/k+1)
            P(k+1/k+1)= P(k+1/k) - K(k+1)*C(k+1)*P(k+1/k)
***************************************************************************/

void metodoParaCalcularCorreccionProbabilidad()
{
/* multiplicacion K(k+1)*C(k+1) = R */
   metodoParaEfectuarMultipicacionDe2x2_2x2(ganancia_K1,ganancia_K2,C1,C2);
   metodoParaCargarResultado2x2();
   
/* multiplicacion R*P(k+1/k)= R2 */
   metodoParaEfectuarMultipicacionDe2x2_2x2(resultado1,resultado2,matriz_P1,matriz_P2);
   metodoParaCargarResultado2x2();
   
/* resta P(k+1/k) - R2 */   
   metodoParaEfectuarResta2x2_2x2(matriz_P1,matriz_P2,resultado1,resultado2);
   metodoParaCargarResultado2x2();
   valor1= metodoParaRegresarelemento1_1()*10000; //nota la probabilidad jamas pasara del valor 10 mil.
   valor2= metodoParaRegresarelemento2_2()*10000;
}

/*****************************************************************************
   metodo para calcular matriz X corregida. x(k+1/k+1)
              x(k+1/k+1)= x(k+1/k)+ K*[y(k) - y(k+1)]
              y(k)-----> es el valor del angulo medido en la siguiente observacion.
              y(k+1)----> es el valor del angulo predicho por el filtro de kalman.
              x(k+1/k)----> matriz X calculada al inicio del filtro, (Prediccion)
*******************************************************************************/
void metodoParaCorregirLaMatriz_X_predicha()
{
   /* Innovacion [y(k) - y(k+1)] */
   metodoParaRestar2x1_2x1(matrizMedida_X,matriz_X);
   resultado1[0]= metodoParaRegresarelemento1_1();
   resultado1[1]= metodoParaRegresarelemento1_2();
   
   //valor3=-1000*resultado1[0];
   //valor4=-1000*resultado1[1];
   
   /* mutiplicacion K*[y(k) - y(k+1)] = R */
   metodoParaEfectuarMultiplicacion2x2_2x1(ganancia_K1,ganancia_K2,resultado1);
   resultado1[0]= metodoParaRegresarelemento1_1();
   resultado1[1]= metodoParaRegresarelemento1_2();
   
   /* Suma x(k+1/k) + R = x(k+1/k+1)*/
   metodoParaSumar2x1_2x1(matriz_X,resultado1);
   valor1= metodoParaRegresarelemento1_1()*100;
   valor2= metodoParaRegresarelemento1_2()*100;
}

/****************************************************************************/
void metodoParaCargarResultado2x2()
{
   resultado1[0]= metodoParaRegresarelemento1_1();
   resultado1[1]= metodoParaRegresarelemento1_2();
   resultado2[0]= metodoParaRegresarelemento2_1();
   resultado2[1]= metodoParaRegresarelemento2_2();   
}

int16 metodoParaRegresarOrientacion()
{
   return valor1;
}
int16 metodoParaRegresarVelocidadAngular()
{
   return valor2;
}

int8 metodoParaRegresarBajo1()
{
   return b1;
}
int8 metodoParaRegresarAlto1()
{
   return al1;
}
int8 metodoParaRegresarBajo2()
{
   return b2;
}
int8 metodoParaRegresarAlto2()
{
   return al2;
}
////////////////////////////////////////////////////////////////////////////
void metodoParaCargarMatriz_X()
{
   valor1=256*read_eeprom(0x09)+(int16)read_eeprom(0x08);
   matriz_X[0]=(float)valor1/100;
   valor2=256*read_eeprom(0x11)+(int16)read_eeprom(0x10);
   matriz_X[1]=(float)valor2/100;
}
void metodoParaGuardarValores_X()
{
   b1= make8(valor1,0);
   al1= make8(valor1,1);
   metodoParaMatriz_X00(b1);
   metodoParaMatriz_X01(al1);
   b2= make8(valor2,0);
   al2= make8(valor2,1);
   metodoParaMatriz_X10(b2);
   metodoParaMatriz_X11(al2);
}

///////////////////////////////////////////////////////////////////////////////
void metodoParaCargarMatriz_P()
{
   valor1=256*read_eeprom(0x02)+(int16)read_eeprom(0x01);
   matriz_P1[0]=(float)valor1/10000;
   valor2=256*read_eeprom(0x06)+(int16)read_eeprom(0x04);
   matriz_P2[1]=(float)valor2/10000;
}

void metodoParaGuardarValores_P()
{
   b1= make8(valor1,0);
   al1= make8(valor1,1);
   metodoParaValor00(b1);
   metodoParaValor01(al1);
   b2= make8(valor2,0);
   al2= make8(valor2,1);
   metodoParaValor10(b2);
   metodoParaValor11(al2);
}
void metodoParaValor00(int8 priValor)
{
   write_eeprom(0x01,priValor);   
}
void metodoParaValor01(int8 segValor)
{
   write_eeprom(0x02,segValor);   
}
void metodoParaValor10(int8 terValor)
{
   write_eeprom(0x04,terValor);   
}
void metodoParaValor11(int8 cuaValor)
{
   write_eeprom(0x06,cuaValor);   
}
void metodoParaMatriz_X00(int8 bajo0)
{
   write_eeprom(0x08,bajo0);   
}
void metodoParaMatriz_X01(int8 alto0)
{
   write_eeprom(0x09,alto0);   
}
void metodoParaMatriz_X10(int8 bajo1)
{
   write_eeprom(0x10,bajo1);   
}
void metodoParaMatriz_X11(int8 alto1)
{
   write_eeprom(0x11,alto1);   
}
