///////////

//Defines//

///////////

#define LIMIAR_SENSORES 300
#define CPD 166
#define CP 55



//#define TESTE_SENSORES

#define PRINCIPAL

//#define MOSTRAR_VALORES



/////////////////

//Dados globais//

////////////////

//Sensores//
int sensor_0 = A0;
 
int sensor_1 = A1;


int sensor_2 = A2;

int sensor_3 = A3;

int sensor_4 = A4;

uint8_t array_sensores = 0;

int leitura0;
int leitura1;

int leitura2;

int leitura3;

int leitura4;

//Compensador PID//

int A = 0.7;

int Kp = CP;

int Ki = 15;

int Kd = CPD;

int resposta_p = 0;

int resposta_i = 0;

int resposta_d = 0;

int Resposta_PID = 0;

int erro_array = 0;

int amostra_atual = 0;

int amostra_anterior = 0;

int erro_atual = 0;

int erro_anterior = 0;

//Velocidade//

int velocidade = 255;

int respPI;

int Vel_up = 255;   // Velocidade de subida

int Vel_med = 200;   // Velocidade no plano

int Vel_down = 70;  // Velocidade de descida

int Vplus, Vless;

int subindo;

int IN0A, IN1A, IN0B, IN1B;

int ENA = 9; // motor esquerdo

int ENB = 10; //motor direito

//declaracao dos motores//

int motor1_a = 4;

int motor1_b = 6;

int motor2_a = 5;

int motor2_b = 7;
//ligacao do robo//
int robo_ligado = 0;
int linha_final;



//////////////////////////////////////////////////////////////////
/////////////////Bloco das funções no plano//////////////////////
/////////////////////////////////////////////////////////////////
void frente(int velocidade) {

  analogWrite(ENA, velocidade );

  analogWrite(ENB, velocidade );



  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , HIGH);

  Serial.print("\t");
  Serial.print(velocidade);

}

//Giro a esquerda desligando um dos motores//

void giro_a_esquerda(int velocidade, int respPI)
{

  Vplus = velocidade + respPI;
  if (Vplus > 255) {
    Vplus = 255;
  } else if (Vplus < 0) {
    Vplus = 0;
  }

  Vless = velocidade - respPI;
  if (Vless > 255) {
    Vless = 255;
  } else if (Vless < 0) {
    Vless = 0;
  }

  analogWrite(ENA, Vplus );

  analogWrite(ENB, Vless );



  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , LOW);

  Serial.print("giro a esquerda");
  Serial.print("\t");
  Serial.print("motor ENA");
  Serial.print("\t");
  Serial.print(Vplus);
  Serial.print("\t");
  Serial.print("motor ENB");
  Serial.print("\t");
  Serial.print(Vless);
}

void curva_esquerda(int velocidade, int respPI)
{

  Vplus = velocidade + respPI;
  if (Vplus > 255) {
    Vplus = 255;
  } else if (Vplus < 0) {
    Vplus = 0;
  }

  Vless = velocidade - respPI;
  if (Vless > 255) {
    Vless = 255;
  } else if (Vless < 0) {
    Vless = 0;
  }

  analogWrite(ENA, Vplus );

  analogWrite(ENB, Vless );

  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , HIGH);

  Serial.print("curva a esquerda");
  Serial.print("\t");
  Serial.print("motor ENA");
  Serial.print("\t");
  Serial.print(Vplus);
  Serial.print("\t");
  Serial.print("motor ENB");
  Serial.print("\t");
  Serial.print(Vless);

}

//Giro a direita desligando um dos motores//

void giro_a_direita(int velocidade, int respPI)

{

  Vplus = velocidade + respPI;
  if (Vplus > 255) {
    Vplus = 255;
  } else if (Vplus < 0) {
    Vplus = 0;
  }

  Vless = velocidade - respPI;
  if (Vless > 255) {
    Vless = 255;
  } else if (Vless < 0) {
    Vless = 0;
  }

  analogWrite(ENA, Vplus ); //neste caso e o menor pois o erro e negativo

  analogWrite(ENB, Vless ); //neste caso e o maior pois o erro e negativo



  digitalWrite(motor1_a , LOW);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , HIGH);

  Serial.print("giro a direita");
  Serial.print("\t");
  Serial.print("motor ENB");
  Serial.print("\t");
  Serial.print(Vless);
  Serial.print("\t");
  Serial.print("motor ENA");
  Serial.print("\t");
  Serial.print(Vplus);

}

void curva_direita(int velocidade, int respPI)
{

  Vplus = velocidade + respPI;
  if (Vplus > 255) {
    Vplus = 255;
  } else if (Vplus < 0) {
    Vplus = 0;
  }

  Vless = velocidade - respPI;
  if (Vless > 255) {
    Vless = 255;
  } else if (Vless < 0) {
    Vless = 0;

  }

  analogWrite(ENA, Vplus ); //neste caso e o menor pois o erro e negativo

  analogWrite(ENB, Vless ); //neste caso e o maior pois o erro e negativo


  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , HIGH);

  Serial.print("curva a direita");
  Serial.print("\t");
  Serial.print("motor ENB");
  Serial.print("\t");
  Serial.print(Vless);
  Serial.print("\t");
  Serial.print("motor ENA");
  Serial.print("\t");
  Serial.print(Vplus);
}


void motores_parados()

{

  Serial.print("vou desligar");

  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , HIGH);

  digitalWrite(motor2_a , HIGH);

  digitalWrite(motor2_b , HIGH);
  Serial.print("\t");
  Serial.print("desliguei");
  delay(5000);

}

void setup() {

  Serial.begin(9600);
  pinMode(13, OUTPUT);
 

}



void loop() {

  leitura0 = analogRead(sensor_0);

  leitura1 = analogRead(sensor_1);

  leitura2 = analogRead(sensor_2);

  leitura3 = analogRead(sensor_3);

  leitura4 = analogRead(sensor_4);


  while ( robo_ligado == 0) {
    delay (1000);
    robo_ligado++;
  }

  //teste sensores//

#ifdef TESTE_SENSORES

  Serial.print(leitura0);

  Serial.print("\t");

  Serial.print(leitura1);

  Serial.print("\t");

  Serial.print(leitura2);

  Serial.print("\t");

  Serial.print(leitura3);

  Serial.print("\t");

  Serial.print(leitura4);

  Serial.print("\t");

  Serial.println();
#endif
/*

horario_agora = millis();
horario_minimo = horario_inicio + tempo_minimo;
*/
#ifdef PRINCIPAL


if (millis() >= 13000  && millis() < 18500){  //USAR UM SEGUNDO A MENOS QUE O DESEJADO
  Kp= 30;
  velocidade = 20;
  digitalWrite(13, HIGH);
} else {
  Kp = CP;
  velocidade = 200;
  digitalWrite(13, LOW);
}
                   //Bloco de montagem das estrutura do array_sensores//
//if ((leitura0 < LIMIAR_SENSORES && leitura1 < LIMIAR_SENSORES && leitura2 < LIMIAR_SENSORES && leitura3 < LIMIAR_SENSORES && leitura4 < LIMIAR_SENSORES) && millis() >= 22000)
  if (millis() >= 19250)
  {
   /* linha_final = millis();
    while(1){

      if((millis() - linha_final) > 9000)
     */
     digitalWrite(13, HIGH);
     delay(400);
     
     {
        motores_parados();
      
    }
    

  } else if (leitura0 > LIMIAR_SENSORES && leitura1 > LIMIAR_SENSORES && leitura2 < LIMIAR_SENSORES && leitura3 > LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b00100;

  } else if (leitura0 > LIMIAR_SENSORES && leitura1 > LIMIAR_SENSORES && leitura2 > LIMIAR_SENSORES && leitura3 < LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b00010;

  } else if (leitura0 > LIMIAR_SENSORES && leitura1 > LIMIAR_SENSORES && leitura2 < LIMIAR_SENSORES && leitura3 < LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b00110;

  } else if (leitura0 > LIMIAR_SENSORES && leitura1 < LIMIAR_SENSORES && leitura2 > LIMIAR_SENSORES && leitura3 > LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b01000;

  } else if (leitura0 > LIMIAR_SENSORES && leitura1 < LIMIAR_SENSORES && leitura2 < LIMIAR_SENSORES && leitura3 > LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b01100;

  } else if (leitura0 < LIMIAR_SENSORES && leitura1 < LIMIAR_SENSORES && leitura2 > LIMIAR_SENSORES && leitura3 > LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b11000;

  } else if (leitura0 < LIMIAR_SENSORES && leitura1 < LIMIAR_SENSORES && leitura2 < LIMIAR_SENSORES && leitura3 > LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b11100;

  } else if (leitura0 < LIMIAR_SENSORES && leitura1 > LIMIAR_SENSORES && leitura2 > LIMIAR_SENSORES && leitura3 > LIMIAR_SENSORES && leitura4 > LIMIAR_SENSORES)
  {

    array_sensores = 0b10000;

  }else if (leitura0 > LIMIAR_SENSORES && leitura1 > LIMIAR_SENSORES && leitura2 < LIMIAR_SENSORES && leitura3 < LIMIAR_SENSORES && leitura4 < LIMIAR_SENSORES)
  {

    array_sensores = 0b00111;

  } else if (leitura0 > LIMIAR_SENSORES && leitura1 > LIMIAR_SENSORES && leitura2 > LIMIAR_SENSORES && leitura3 < LIMIAR_SENSORES && leitura4 < LIMIAR_SENSORES)
  {

    array_sensores = 0b00011;

  } else if (leitura0 > LIMIAR_SENSORES && leitura1 > LIMIAR_SENSORES && leitura2 > LIMIAR_SENSORES && leitura3 > LIMIAR_SENSORES && leitura4 < LIMIAR_SENSORES)
  {

    array_sensores = 0b00001;

  }

  //Calculo dos erros//

  if (array_sensores == 0b01000) {

    erro_array = 2;

  } else if (array_sensores == 0b00100) {

    erro_array = 0;


  } else if (array_sensores == 0b00010) {

    erro_array = -2;

  } else if (array_sensores == 0b00011 ) {

    erro_array = -3;


  } else if (array_sensores == 0b00110) {

    erro_array = -1;


  } else if (array_sensores == 0b01100) {

    erro_array = 1;


  } else if (array_sensores == 0b10000) {

    erro_array = 4;


  } else if (array_sensores == 0b11000) {

    erro_array = 3;


  } else if (array_sensores == 0b00001) {

    erro_array = -4;


  } else if (array_sensores == 0b11100) {

    erro_array = 5;


  } else if (array_sensores == 0b00111) {

    erro_array = -5;


  }




#ifdef MOSTRAR_VALORES

  Serial.print("Valor do erro:");

  Serial.print("  ");

  Serial.print(erro_array);

  Serial.print("  ");

#endif

  //Compensador PID//

  erro_atual = erro_array;

  resposta_p = erro_atual * Kp;

  resposta_i = resposta_i + erro_atual * Ki * A;

  resposta_d = Kd * (erro_atual - erro_anterior);

  //Resposta_PID = resposta_p + resposta_i + resposta_d;
  Resposta_PID = resposta_p + resposta_d;
  //Resposta_PID = resposta_p + resposta_i;

  erro_anterior = erro_atual;

#ifdef MOSTRAR_VALORES

  Serial.print("\t");

  Serial.print("Resposta PID:");

  Serial.print("  ");

  Serial.print(Resposta_PID);

  Serial.print("  ");

#endif


  
  if (robo_ligado >= 3) {
    motores_parados();
  }

  //PRINCIPAL//

  if (erro_array == 0) {

    frente(velocidade);

  }

  if (erro_array == 1) {

    //giro_a_esquerda(velocidade, Resposta_PID); // erro positivo virar à esqerda
    curva_esquerda(velocidade, Resposta_PID);
  }

  if (erro_array == 2) {

    //giro_a_esquerda(velocidade, Resposta_PID);
    curva_esquerda(velocidade, Resposta_PID);

  } if (erro_array == 3) {

    //giro_a_esquerda(velocidade, Resposta_PID);
    curva_esquerda(velocidade, Resposta_PID);

  } if (erro_array == 4) {

    //giro_a_esquerda(velocidade, Resposta_PID);
    curva_esquerda(velocidade, Resposta_PID);

  } if (erro_array == 5) {

    giro_a_esquerda(velocidade, Resposta_PID);
    //curva_esquerda(velocidade, Resposta_PID);

  }

  if (erro_array == -1) {

    //giro_a_direita(velocidade, Resposta_PID); // erro negativo virar à esquerda
    curva_direita(velocidade, Resposta_PID);
  }

  if (erro_array == -2) {

    //giro_a_direita(velocidade, Resposta_PID);
    curva_direita(velocidade, Resposta_PID);

  } if (erro_array == -3) {

    //giro_a_direita(velocidade, Resposta_PID);
    curva_direita(velocidade, Resposta_PID);

  } if (erro_array == -4) {

    //giro_a_direita(velocidade, Resposta_PID);
    curva_direita(velocidade, Resposta_PID);

  } if (erro_array == -5) {

    giro_a_direita(velocidade, Resposta_PID);
    //curva_direita(velocidade, Resposta_PID);

  }



#ifdef MOSTRAR_VALORES

  Serial.print("\t");

  Serial.print("V+:");

  Serial.print("  ");

  Serial.print(Vplus);

  Serial.print("  ");

  Serial.print("\t");

  Serial.print("V-:");

  Serial.print("  ");

  Serial.println(Vless);

#endif



#endif


}
