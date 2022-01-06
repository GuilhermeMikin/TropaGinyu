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
int sensor_0 = A1;
 
//int sensor_1 = A1;

int sensor_2 = A2;

int sensor_3 = A3;

int sensor_4 = A4;

uint8_t array_sensores = 0;

int leitura0;
int leitura1;
int leitura2;
int leitura3;
int leitura4;


//////////////////////////////////////////////////////////////////
/////////////////Bloco das funções no plano//////////////////////
/////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);
//  pinMode(13, OUTPUT);

}

void loop() {

  leitura0 = analogRead(sensor_0);
  Serial.print(leitura0);
  Serial.println();
  delay(1000);
//  leitura1 = analogRead(sensor_1);
//
//  leitura2 = analogRead(sensor_2);
//
//  leitura3 = analogRead(sensor_3);
//
//  leitura4 = analogRead(sensor_4);

  }

  //teste sensores//

#ifdef TESTE_SENSORES

//  Serial.print(leitura0);
//
//  Serial.print("\t");

//  Serial.print(leitura1);
//
//  Serial.print("\t");
//
//  Serial.print(leitura2);
//
//  Serial.print("\t");
//
//  Serial.print(leitura3);
//
//  Serial.print("\t");
//
//  Serial.print(leitura4);
//
//  Serial.print("\t");

  Serial.println();
#endif
