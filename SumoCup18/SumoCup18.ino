
#include <Ultrasonic.h>



//Define os pinos para o trigger e echo

#define pino_trigger 11

#define pino_echo 3

 

Ultrasonic ultrasonic(pino_trigger, pino_echo);



// botão de inicialização

bool robo_ligado = false;

//int pino_do_btn = 1;

int led = 12;

int buz = A5;



//botão de inicialização

//sensores

int S1 = 0; 

int S2 = 0; 

int S3 = 0; 

int S4 = 0; 



//declaracao dos motores

int motor1_a=4;

int motor1_b=2;

int motor2_a=6;

int motor2_b=7;



//pinos pwm

int controle_A = 3;

int controle_B = 5;



void procura ()

{

  /*faz um giro no proprio eixo no sentido antihorário*/

   analogWrite(controle_A, 45 );

   analogWrite(controle_B, 55 );

        

        digitalWrite(motor1_a , HIGH);

        digitalWrite(motor1_b , LOW);

        digitalWrite(motor2_a , HIGH);

        digitalWrite(motor2_b , LOW);



}



void ataque(float olhos)

{

  /* faz um movimento para frente*/

  if (olhos < 15 && olhos > 7)

    {

   analogWrite(controle_A, 60 );

   analogWrite(controle_B,65 );

   

        digitalWrite(motor1_a , HIGH);

        digitalWrite(motor1_b , LOW);

        digitalWrite(motor2_a , LOW);

        digitalWrite(motor2_b , HIGH);

    }

 

    if (olhos < 6.99)

    {

     analogWrite(controle_A, 248 );

   analogWrite(controle_B,255 );

   

        digitalWrite(motor1_a , HIGH);

        digitalWrite(motor1_b , LOW);

        digitalWrite(motor2_a , LOW);

        digitalWrite(motor2_b , HIGH);

    }

}



void voltar_ao_campo( int S1, int S2, int S3, int S4)

{

  

   /* se s1,s2 retornarem valor faz um movimento de ré por 1 segundo*/ 

 if ((S1<930)&&(S2<930)){

   analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   

        digitalWrite(motor1_a , LOW);

        digitalWrite(motor1_b , HIGH);

        digitalWrite(motor2_a , HIGH);

        digitalWrite(motor2_b , LOW);

      //  delay(1000);

}

   /* se s1 retorna movimento ré*/

 if(S1 < 930)

 {

     analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   

        digitalWrite(motor1_a , LOW);

        digitalWrite(motor1_b , HIGH);

        digitalWrite(motor2_a , HIGH);

        digitalWrite(motor2_b , LOW);

      //  delay(1000);

   }

   /* se s2 movimento ŕe*/

 if(S2<930)

 {

     analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   

        digitalWrite(motor1_a , LOW);

        digitalWrite(motor1_b , HIGH);

        digitalWrite(motor2_a , HIGH);

        digitalWrite(motor2_b , LOW);

      //  delay(1000);

   }

   /* se s3 movimento frente*/

   if(S3<930)

   {

     analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   ;

        digitalWrite(motor1_a , HIGH);

        digitalWrite(motor1_b , LOW);

        digitalWrite(motor2_a , LOW);

        digitalWrite(motor2_b , HIGH);

       // delay(1000);

   }

  // virar a esquerda
  
   if ((S1<930)&&(S3<930)){

   analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   

        digitalWrite(motor1_a , LOW);

        digitalWrite(motor1_b , HIGH);

        digitalWrite(motor2_a , LOW);

        digitalWrite(motor2_b , HIGH);

      //  delay(1000);

}


  // virar a direita
  
   if ((S2<930)&&(S4<930)){

   analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   

        digitalWrite(motor1_a , HIGH);

        digitalWrite(motor1_b , LOW);

        digitalWrite(motor2_a , HIGH);

        digitalWrite(motor2_b , LOW);

      //  delay(1000);

}

if ((S3<930)&&(S4<930)){

   analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   

        digitalWrite(motor1_a , HIGH);

        digitalWrite(motor1_b , LOW);

        digitalWrite(motor2_a , LOW);

        digitalWrite(motor2_b , HIGH);

      //  delay(1000);

}

   if(S4<930)

   {

     analogWrite(controle_A, 80 );

   analogWrite(controle_B,80 );

   ;

        digitalWrite(motor1_a , HIGH);

        digitalWrite(motor1_b , LOW);

        digitalWrite(motor2_a , LOW);

        digitalWrite(motor2_b , HIGH);

       // delay(1000);

   }
   /* se nenhum sensor, procurar*/



}



void setup() {

  Serial.begin(9600);

  // put your setup code here, to run once:

  
  pinMode(pino_do_btn, INPUT_PULLUP);

  pinMode(led , OUTPUT);

  pinMode(buz , OUTPUT);

  

    // configuracoes dos motores

   pinMode (motor1_a , OUTPUT);

   pinMode (motor1_b , OUTPUT);

   pinMode (motor2_a , OUTPUT);

   pinMode (motor2_b , OUTPUT);

  

}



void loop() {

  // put your main code here, to run repeatedly:

 // int estado = (!digitalRead(pino_do_btn));

//  Serial.println(estado);

//if(!digitalRead(pino_do_btn)){

robo_ligado = true;

delay (6000);

digitalWrite(buz, HIGH);

delay (1000);

digitalWrite(buz, LOW);


if(robo_ligado){

//seu código aqui



float olhos;

long microsec = ultrasonic.timing(); 

olhos = ultrasonic.convert(microsec, Ultrasonic::CM);



    S1 = analogRead(A0);

    if (S1<930){

    Serial.println("sensor1 ");

    Serial.println(S1);

    }

    S2 = analogRead(A2);

    if (S2<930){

   Serial.println("sensor2 ");

    Serial.println(S2);

     }

    S3 = analogRead(A1); 

    if (S3<970){

    Serial.println("sensor3 ");

    Serial.println(S3);

        }

    S4 = analogRead(A3); 

    if (S4<970){

    Serial.println("sensor4 ");

    Serial.println(S4);

        }    

 

   if(S1 > 930 && S2 > 930 && S3 > 930 && S4 > 930)

   {

      Serial.println(olhos);

      if (olhos > 15)

      {

        procura();

      }else if(olhos < 15)

      {

        ataque(olhos);

      }

   }

      else if(S1 < 930 || S2 < 930 || S3 < 930 || S4 < 930) 

    {

      voltar_ao_campo(S1,S2,S3,S4);

    }

    

delay(20);



}

} 
