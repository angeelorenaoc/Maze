// C++ code
//Sensores
#define Pecho1 12
#define Ptrig1 13
#define Pecho2 10
#define Ptrig2 11
#define Pecho3 8
#define Ptrig3 9

//Var init
unsigned int Maze_X = 4;
unsigned int Maze_Y = 6;
unsigned int Goal[] = {3,7}; //
unsigned int pos_act[2] = {7,3};

/*
E N O S default
0 norte
1 oeste
2 sur
3 este
*/
unsigned int dir_init = 0; 

unsigned int Tam_MazeX = (2*Maze_X)+1;
unsigned int Tam_MazeY = (2*Maze_Y)+1;

int Maze[13][9];

//Variables del auto
int direcciones[4][2] = {{-1,0},{0,-1},{1,0},{0,1}};// E N O S

//umbrales
unsigned int Umbral[] = {20,20,20};

char option = ' ';
double duracion, distancia,dl,dc,dr;

void setup() {
  
  Serial.begin(9600);
   pinMode(Pecho1, INPUT);     // define el pin 6 como entrada (echo)
  pinMode(Ptrig1, OUTPUT);    // define el pin 7 como salida  (triger)
  
  pinMode(Pecho2, INPUT);     
  pinMode(Ptrig2, OUTPUT);
  
  pinMode(Pecho3, INPUT);
  pinMode(Ptrig3, OUTPUT);
  
  //Serial.println("Iniciando");
  init_Maze();
  init_dir();
  pr_Maze(0);
  	
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Continuar");
  //Codogo y calibraci

  if(Serial.available() != 0){
    option = Serial.read();
    //1. Verificar paredes
  	ActParedes();
    pr_Maze(1);
    Serial.println("----------------");
    
    //2. Moverse
    Mov();
    
    //implab
    pr_Maze(1);
    Serial.println("----------------");
    Serial.print(pos_act[0]);
    Serial.print("-");
    Serial.println(pos_act[1]);
    Serial.println("----------------");
  }
  delay(100);
}



void init_Maze(){
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
  for(int shift=0;shift<dir_init;shift++){
    girar_dir(1);
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
  dl = MedirDist(Ptrig1,Pecho1); // Distancia del sensor izq
  dc = MedirDist(Ptrig2,Pecho2); // Distancia del sensor centro
  dr = MedirDist(Ptrig3,Pecho3); // Distancia del sensor der
  int dis[3] = {dl,dc,dr};
  for(int i=0;i<3;i++){
    if(dis[i]<Umbral[i]){
      Maze[pos_act[1]+direcciones[i][1]][pos_act[0]+direcciones[i][0]]=1;
    }
    else{
      Maze[pos_act[1]+direcciones[i][1]][pos_act[0]+direcciones[i][0]]=0;
    }
  }
}

//Movimiento
void Mov(){
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
    girar_dir(-1);
    break;
    case 1://no girar
    girar_dir(0);
    break;
    case 2://derecha
    girar_dir(1);
    break;
    case 3://devolverse
    girar_dir(1);
    girar_dir(1);
    //penalizar
    Maze[pos_act[1]][pos_act[0]]+=2;
    break;
  }
  //Ejecutar desplazamiento
  pos_act[1] += 2*direcciones[1][1];
  pos_act[0] += 2*direcciones[1][0];
  
}

void girar_dir(int sentido){
  int Xcambio[2];
  switch(sentido){
    case(1)://derecha
    Xcambio[0]=direcciones[0][0];
    Xcambio[1]=direcciones[0][1];
    // corrimiento hacia la derecha
  	for(int i=0;i<3;i++){
      
      direcciones[i][0] = direcciones[i+1][0];
      direcciones[i][1] = direcciones[i+1][1];
  	}
    direcciones[3][0] = Xcambio[0];
    direcciones[3][1] = Xcambio[1];
    break;
    
    case(-1)://izq
    Xcambio[0] = direcciones[3][0]; 
    Xcambio[1] = direcciones[3][1]; 
    // corrimiento hacia la izq
    for(int i=3;i>0;i--){
      direcciones[i][0] = direcciones[i-1][0];
      direcciones[i][1] = direcciones[i-1][1];
    }
    direcciones[0][0] = Xcambio[0];
    direcciones[0][1] = Xcambio[1];
    break;
    default:
    //No se actualizan
    break;
  }
}
double MedirDist(int sensorTrig, int sensorEcho)
{
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(4);
  digitalWrite(sensorTrig, HIGH);   // genera el pulso de triger por 10us
  delayMicroseconds(10);
  digitalWrite(sensorTrig, LOW);
  duracion = pulseIn(sensorEcho, HIGH);
  distancia = (duracion/2) / 29;   
  return distancia;
}