int ENA = 9; // motor esquerdo

int ENB = 10; //motor direito

//declaracao dos motores//
int velocidade;

int respPI;

int Vel_up = 255;   // Velocidade de subida

int Vel_med = 90;   // Velocidade no plano

int Vel_down = 80;  // Velocidade de descida

int Vplus, Vless;

int IN0A, IN1A, IN0B, IN1B;

int motor1_a = 4;

int motor1_b = 6;

int motor2_a = 5;

int motor2_b = 7;

void frente() {

  analogWrite(ENA, 255 );

  analogWrite(ENB, 255 );



  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , HIGH);

}



//Giro a esquerda desligando um dos motores//

void giro_a_esquerda(int velocidade, int respPI)

{

  velocidade = 130;

  Vplus = velocidade + respPI;

  Vless = velocidade - respPI;

  analogWrite(ENA, Vplus );

  analogWrite(ENB, Vless );



  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , LOW);

  // delay(2000);

}

//Giro a direita desligando um dos motores//

void giro_a_direita(int velocidade, int respPI)

{

  velocidade = 130;

  Vplus = velocidade + respPI;

  Vless = velocidade - respPI;

  analogWrite(ENA, Vless );

  analogWrite(ENB, Vplus );



  digitalWrite(motor1_a , LOW);

  digitalWrite(motor1_b , LOW);

  digitalWrite(motor2_a , LOW);

  digitalWrite(motor2_b , HIGH);

  // delay(2000);

}



void motores_parados()

{

  digitalWrite(motor1_a , HIGH);

  digitalWrite(motor1_b , HIGH);

  digitalWrite(motor2_a , HIGH);

  digitalWrite(motor2_b , HIGH);

  //  delay(1000);

}
void setup() {
}
  // put your setup code here, to run once:

void loop() {
  // put your main code here, to run repeatedly:
delay (2000);
frente();
delay (2000);
giro_a_direita(1,250);
delay (2000);
giro_a_esquerda(1,250);
delay (2000);
motores_parados();
delay (2000);
}
