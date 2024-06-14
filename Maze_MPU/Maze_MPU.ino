
//Pines digitales asignados para los sensores ultrasonido
#define Pecho1 27
#define Ptrig1 26
#define Pecho2 29
#define Ptrig2 28
#define Pecho3 31
#define Ptrig3 30

#include "Wire.h"

//Librería usada para el control del shield
#include <AFMotor.h>

//Deltas de los ángulos para determinar el ragngo de giro
#define delta_ang_inf 1
#define delta_ang_sup 1

//Tamaño del laberinto
unsigned int Maze_X = 4;
unsigned int Maze_Y = 6;

//Meta
unsigned int Goal[] = {3,7}; 

//Posicion inicial en la matriz y su dirección
unsigned int pos_act[2] = {7,3};
unsigned int dir_init = 0; 

//Construcción de la matriz teniendo en cuenta las celdas las cuales son asignadas a las paredes
unsigned int Tam_MazeX = (2*Maze_X)+1;
unsigned int Tam_MazeY = (2*Maze_Y)+1;

int Maze[13][9];

double dist = 0;

//Motores
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);  

//Velocidades
int motorSpeed = 255;
int motorSpeed_Giro = 110;

//Bandera que da el primer impulso al motor
bool band = 1;
//Bandera para la correccion del giro
bool cor_giro = 0;

//Varible que indica la opción en el case de giros
int giro = 0;

//MPU 
int MPU_addr = 0x68;

int cal_gyro = 1;  //set to zero to use gyro calibration offsets below.

float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { 0., 0., 0.}; //raw offsets, determined for gyro at rest
#define gscale ((250./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

float q[4] = {1.0, 0.0, 0.0, 0.0};

unsigned long now_ms, last_ms = 0; //millis() timers

// print interval
unsigned long print_ms = 200; //print angles every "print_ms" milliseconds
float yaw, pitch, roll; //Euler angle output

//Matriz con el rango de direcciones de giro
int direccionesRan[4][2] = {{90-delta_ang_inf,90+delta_ang_sup},{360-delta_ang_inf,delta_ang_sup},{270-delta_ang_inf,270+delta_ang_sup},{180-delta_ang_inf,180+delta_ang_sup}};

//Variables de dirección del auto
/*
E N O S default
0 norte
1 oeste
2 sur
3 este
*/
int direcciones[4][2] = {{-1,0},{0,-1},{1,0},{0,1}};// E N O S

//umbrales de distancia
unsigned int Umbral[] = {15,15,15};

char option = ' ';

double duracion, distancia,dl,dc,dr;


void setup() {

  Serial.begin(9600);

  //Ultrasonidos
  //Izquierda
  pinMode(Pecho1,INPUT);
  pinMode(Ptrig1,OUTPUT);
  //Centro
  pinMode(Pecho2,INPUT);
  pinMode(Ptrig2,OUTPUT);
  //Derecha
  pinMode(Pecho3,INPUT);
  pinMode(Ptrig3,OUTPUT);
  
  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop() {

  //Codigo y calibración mpu
  static unsigned int i = 0; //loop counter
  static float deltat = 0;  //loop time in seconds
  static unsigned long now = 0, last = 0; //micros() timers
  static long gsum[3] = {0};
  //raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp; //temperature

  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read() << 8;
  az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read() << 8;
  gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // calibrate gyro upon startup. SENSOR MUST BE HELD STILL (a few seconds)
  i++;
  if (cal_gyro) {
    gsum[0] += gx; gsum[1] += gy; gsum[2] += gz;
    if (i == 500) {
      cal_gyro = 0;  //turn off calibration and print results
      for (char k = 0; k < 3; k++) G_off[k] = ((float) gsum[k]) / 500.0;

      Serial.print("G_Off: ");
      Serial.print(G_off[0]);
      Serial.print(", ");
      Serial.print(G_off[1]);
      Serial.print(", ");
      Serial.print(G_off[2]);
      Serial.println();
      init_Maze();
      init_dir();
      pr_Maze(0);  	
    }
  }
  // normal AHRS calculations

  else {

    //Estimación de aceleraciones, giros y ángulos
    Axyz[0] = (float) ax;
    Axyz[1] = (float) ay;
    Axyz[2] = (float) az;

    //apply offsets and scale factors from Magneto
    for (i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float) gy - G_off[1]) * gscale;
    Gxyz[2] = ((float) gz - G_off[2]) * gscale;

    //  snprintf(s,sizeof(s),"mpu raw %d,%d,%d,%d,%d,%d",ax,ay,az,gx,gy,gz);
    //  Serial.println(s);

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;


    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
    yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI;
    if (yaw < 0) yaw += 360.0; //compass circle
    //ccrrect for local magnetic declination here
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    now_ms = millis(); //time to print?
    if (now_ms - last_ms >= print_ms) {
      last_ms = now_ms;
      // print angles for serial plotter...
      //  Serial.print("ypr ");
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.println(giro);
      
    }


  //Laberinto
  
    switch(giro){
    case(0):
      //1. Verificar paredes
      if ((pos_act[0] == Goal [0]) && (pos_act[1] == Goal [1])){
        motor1.run(RELEASE);
        motor1.setSpeed(motorSpeed);
        motor2.run(RELEASE);
        motor2.setSpeed(motorSpeed + 60);
        motor3.run(RELEASE);
        motor3.setSpeed(motorSpeed);
        motor4.run(RELEASE);
        motor4.setSpeed(motorSpeed + 40);
        while(1){}
      }
      ActParedes();
      //Actualización de la matriz y dirección de giro
      Mov();
      delay(1000);
    break;
    case(1):
    //Giro derecha
    if((direccionesRan[2][0] == 360 - delta_ang_inf && !(yaw >= direccionesRan[2][0] || yaw <= direccionesRan[2][1])) ||
    (!(direccionesRan[2][0] == 360 - delta_ang_inf) && !(yaw >= direccionesRan[2][0] && yaw <= direccionesRan[2][1]))){
        motor1.run(BACKWARD);
        motor1.setSpeed(motorSpeed+30);
        motor2.run(FORWARD);
        motor2.setSpeed(motorSpeed + 40);
        motor3.run(FORWARD);
        motor3.setSpeed(motorSpeed);
        motor4.run(BACKWARD);
        motor4.setSpeed(motorSpeed + 10);
      //Serial.write(giro)
      }
    else{
        //Serial.write(4);
        
        motor1.run(RELEASE);

        motor2.run(RELEASE);

        motor3.run(RELEASE);

        motor4.run(RELEASE);
        girar_dir(1, direccionesRan);
        giro=3;
        cor_giro=1;
      // offset += 90;
      }
      band = 0;
        if (!band){ 
          motorSpeed = motorSpeed_Giro;
          }
    break;
    case(2):
    //Giro izquierda
    if((direccionesRan[0][0] == 360-delta_ang_inf && !(yaw >= direccionesRan[0][0] || yaw <= direccionesRan[0][1])) ||
    (!(direccionesRan[0][0] == 360-delta_ang_inf) && !(yaw >= direccionesRan[0][0] && yaw <= direccionesRan[0][1]))){
        motor1.run(FORWARD);
        motor1.setSpeed(motorSpeed);
        motor2.run(BACKWARD);
        motor2.setSpeed(motorSpeed + 40);
        motor3.run(BACKWARD);
        motor3.setSpeed(motorSpeed+10);
        motor4.run(FORWARD);
        motor4.setSpeed(motorSpeed + 40);
      //Serial.write(giro)
      }
    else{
        //Serial.write(4);
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        motor3.run(RELEASE);
        motor4.run(RELEASE);
        girar_dir(-1,direccionesRan);
        cor_giro=1;
        giro=3;
      // offset += 90;
      }
      band = 0;
        if (!band){ 
          motorSpeed = motorSpeed_Giro;
          }
    break;
    case(3):
    //Ir hacia adelante
        motorSpeed = 170;
        motor1.run(BACKWARD);
        motor1.setSpeed(motorSpeed);
        motor2.run(BACKWARD);
        motor2.setSpeed(motorSpeed + 50);
        motor3.run(BACKWARD);
        motor3.setSpeed(motorSpeed);
        motor4.run(BACKWARD);
        motor4.setSpeed(motorSpeed + 10);
        delay(580);
        giro=0;
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        motor3.run(RELEASE);
        motor4.run(RELEASE);
    break;
    case(4):
    //Giro 180 grados
    if((direccionesRan[3][0] == 360-delta_ang_inf && !(yaw >= direccionesRan[3][0] || yaw <= direccionesRan[3][1])) ||
    (!(direccionesRan[3][0] == 360-delta_ang_inf) && !(yaw >= direccionesRan[3][0] && yaw <= direccionesRan[3][1]))){
        
        motor1.run(FORWARD);
        motor1.setSpeed(motorSpeed);
        motor2.run(BACKWARD);
        motor2.setSpeed(motorSpeed + 60);
        motor3.run(BACKWARD);
        motor3.setSpeed(motorSpeed);
        motor4.run(FORWARD);
        motor4.setSpeed(motorSpeed + 30);
    }
  else{
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
      girar_dir(1,direccionesRan);
      girar_dir(1,direccionesRan);
      giro=3;
    }
    band = 0;
      if (!band){ 
        motorSpeed = motorSpeed_Giro;
        }
  break;
    }
  }
}

void init_Maze(){
  //Función que delimita la matriz y estbale que las casillas impares son parte del camino y las pares paredes
  int dx, dy,x;
  for(int j = 0; j<Tam_MazeY; j++){
    for(int i = 0; i<Tam_MazeX; i++){
      dy = j-Goal[1];
      //Verificar si esta en una celda de distancia-pared-nada
      if(i%2==1 and j%2==1){ //Si ambos son pares es una celda de distancia
        dx = i-Goal[0];
        x = (abs(dx)+abs(dy))/2; //distancia
    	}
      else{ //paredes
        x=0;
      }
      Maze[j][i] = x;
  	}
  }
}
void init_dir(){
  //Definición de la dirección inicial
  for(int shift=0;shift<dir_init;shift++){
    girar_dir(1,direcciones);
  } 
}

void pr_Maze(bool all){

  for(int j = 0; j<Tam_MazeY; j++){
    for(int i = 0; i<Tam_MazeX; i++){
      if(all){
        Serial.print(Maze[j][i]);
      	Serial.print("~");
      }
      else if(i%2==1 && j%2==1){
        Serial.print(Maze[j][i]);
      	Serial.print("~");
      }
    }
    if(all)Serial.println("/");
    else if(j%2==1)Serial.println("/");
  }
}

void ActParedes(){

  //Función que detecta y actualiza las paredes en la matriz

  dl = MedirDist(Ptrig1,Pecho1); // Distancia del sensor izq
  dc = MedirDist(Ptrig2,Pecho2); // Distancia del sensor centro
  dr = MedirDist(Ptrig3,Pecho3); // Distancia del sensor der
  Serial.print(dl);
  Serial.print(", ");
  Serial.print(dc);
  Serial.print(", ");
  Serial.println(dr);
  int dis[3] = {dl,dc,dr};
  for(int i=0;i<3;i++){
    if(dis[i]<Umbral[i]|| dis[i]==0){
      Maze[pos_act[1]+direcciones[i][1]][pos_act[0]+direcciones[i][0]]=1;
    }
    else{
      Maze[pos_act[1]+direcciones[i][1]][pos_act[0]+direcciones[i][0]]=0;
    }
  }
}


void Mov(){

  //Función que determina el proximo movimiento de acuerdo a las paredes detectadas y el peso de las celdas 

  unsigned int best_dir[3][2]={{0,0},{0,0},{0,0}};
  unsigned int pared,dir_gir;
  unsigned int cam = 0; //ver si hay caminos posible
  unsigned int celda_prox;
  //buscar la mejor opccion
  for(int dir=0;dir<3;dir++){
    pared = Maze[pos_act[1]+direcciones[dir][1]][pos_act[0]+direcciones[dir][0]];
    celda_prox = Maze[pos_act[1]+direcciones[dir][1]*2][pos_act[0]+direcciones[dir][0]*2];
 
    if(pared==0){//no hay pared guardar casilla
      best_dir[cam][0]= celda_prox;
      best_dir[cam][1]= dir;
      cam +=1;
    }
  }
  //decidir mejor camino
  if(cam==0){
    dir_gir = 3;
  }
  else if(cam == 1){
    dir_gir = best_dir[0][1];
  }
  else{
    celda_prox = best_dir[0][0];
    dir_gir = best_dir[0][1];
    for(int i=1;i<cam;i++){
      if(best_dir[i][0]<celda_prox){
        celda_prox = best_dir[i][0];
        dir_gir = best_dir[i][1];
      }
    }
  }
  //Girar segun el caso
  Maze[pos_act[1]][pos_act[0]]+=1;
  switch(dir_gir){
    case 0://izq
    girar_dir(-1,direcciones);
    giro = 2;
    break;
    case 1://no girar
    girar_dir(0,direcciones);
    giro = 3;
    break;
    case 2://derecha
    girar_dir(1,direcciones);
    giro = 1;
    break;
    case 3://devolverse
    girar_dir(1,direcciones);
    girar_dir(1,direcciones);
    giro = 4;
    //penalizar
    Maze[pos_act[1]][pos_act[0]]+=2;
    break;
  }
  //Ejecutar desplazamiento
  pos_act[1] += 2*direcciones[1][1];
  pos_act[0] += 2*direcciones[1][0];
  
}

void girar_dir(int sentido, int lista[4][2]){
  
  //Función encargada de actualizar las listas de direcciones y rangos de los ángulos

  int Xcambio[2];
  switch(sentido){
    case(1)://derecha
    Xcambio[0]=lista[0][0];
    Xcambio[1]=lista[0][1];
    // corrimiento hacia la derecha
  	for(int i=0;i<3;i++){
      
      lista[i][0] = lista[i+1][0];
      lista[i][1] = lista[i+1][1];
  	}
    lista[3][0] = Xcambio[0];
    lista[3][1] = Xcambio[1];
    break;
    
    case(-1)://izq
    Xcambio[0] = lista[3][0]; 
    Xcambio[1] = lista[3][1]; 
    // corrimiento hacia la izq
    for(int i=3;i>0;i--){
      lista[i][0] = lista[i-1][0];
      lista[i][1] = lista[i-1][1];
    }
    lista[0][0] = Xcambio[0];
    lista[0][1] = Xcambio[1];
    break;
    default:
    //No se actualizan
    break;
  }
}

double MedirDist(int sensorTrig, int sensorEcho)
{
  // Función que mide las distancias detectadas por los ultrasonidos
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(4);
  digitalWrite(sensorTrig, HIGH);   // genera el pulso de triger por 10us
  delayMicroseconds(10);
  digitalWrite(sensorTrig, LOW);
  duracion = pulseIn(sensorEcho, HIGH);
  distancia = (duracion/2) / 29;   
  return distancia;
}

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // removed unused accelerometer terms
  
  // Integrate rate of change of quaternion, given by gyro term
  // rate of change = current orientation quaternion (qmult) gyro rate

  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  //add qmult*delta_t to current orientation
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}
