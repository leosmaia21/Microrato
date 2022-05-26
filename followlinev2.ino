#define IR5 A0
#define IR4 A1
#define IR3 A2
#define IR2 A3
#define IR1 A4
#define startBut 8

#define AIN1 5
#define AIN2 3
#define BIN1 9
#define BIN2 6
#define SAMP_FREQ 170;
#define K 25  // pi com k de 3
#define Ti 6  // pi com Ti de 0.5
#define Td 0.5
#define N 20.0


int baseSpeed = 60;
void Motors_Init();
void MotorEsqSpeed(int Speed);
void MotorDirSpeed(int Speed);
void MotorsSpeed(int Vel_Esq, int Vel_Dir);

// Preto = 0
// Branco = 1

int AN[5] = {0, 0, 0, 0, 0};

int minimum = 500;
int maximum = 900;

int LL = 0;
int L = 0;
int M = 0;
int R = 0;
int RR = 0;

int buttonState = LOW;
const float h = 1.0 / SAMP_FREQ;

float u = 0;
float prevError = 0;
float prevPrevError = 0;
float prevU = 0;
float prevPrevU = 0;
float error = 0.0;
/*controlador PI*/
float s0 = K * (1 + (h / Ti));
float s1 = -K;

/*controlador PID*/
/*float X=-(Td/(N*h+Td));
float s0=K*(1+(h/Ti)-X*N);
float s1=K*(X*(1+(h/Ti)+2*N)-1);
float s2= -K*X*(1+N);
float r1= X;*/

// const int startBut = 8;

void setup() {
    IR_Init();
    Motors_Init();

    MotorsSpeed(0, 0);
    buttonState = LOW;

    do {
        buttonState = digitalRead(startBut);
    } while (buttonState == LOW);
    // MotorsSpeed(40, 40);

    /* Initialize Serial with 9600bps  */
    Serial.begin(9600);
}
int esq = 0;
int dir = 0;
int setpoint = 0.0;
void loop() {
    // read line
    AN[0] = analogRead(IR5);
    AN[1] = analogRead(IR4);
    AN[2] = analogRead(IR3);
    AN[3] = analogRead(IR2);
    AN[4] = analogRead(IR1);

    RR = black_white(AN[0], minimum, maximum);
    R = black_white(AN[1], minimum, maximum);
    M = black_white(AN[2], minimum, maximum);
    L = black_white(AN[3], minimum, maximum);
    LL = black_white(AN[4], minimum, maximum);

    // Define se o valor que está a ler corresponde a 1 ou 0 (preto ou branco)

    /*Serial.print(AN[4]);
    Serial.print(" ");
    Serial.print(AN[3]);
    Serial.print(" ");
    Serial.print(AN[2]);
    Serial.print(" ");
    Serial.print(AN[1]);
    Serial.print(" ");
    Serial.println(AN[0]);

    Serial.println("LL L M R RR ");

    Serial.print(LL);
    Serial.print(" ");
    Serial.print(L);
    Serial.print(" ");
    Serial.print(M);
    Serial.print(" ");
    Serial.print(R);
    Serial.print(" ");
    Serial.println(RR);*/

    /*if(RR==1 || LL==1){
      baseSpeed=50;
    }else{
      baseSpeed=70;
    }*/

    error = setpoint - ((LL * (-8.0) + L * (-2.0) + M * 0 + R * 2.0 + RR * 8.0) / 4.0);
    u = prevU + s0 * error + s1 * prevError;
    // u=error*K;

    // u=(1-r1)*prevU + r1*prevPrevU + s0*error + s1*prevError + s2*prevPrevError;
    prevPrevError = prevError;
    prevPrevU = prevU;
    prevU = u;
    prevError = error;

    esq = baseSpeed - u;
    dir = baseSpeed + u;
    Serial.print("\r\n");
    Serial.print("u:");

    Serial.println(u);
    Serial.print("esq:");
    Serial.print(esq);
    Serial.print("dir:");
    Serial.print(dir);
    Serial.println(u);
    Serial.print("esq:");
    Serial.print(esq);
    Serial.print("dir:");
    Serial.print(dir);
    if (esq > 70) {
        esq = 70;
    }
    if (dir > 70) {
        dir = 70;
    }

    MotorsSpeed(esq, dir);

    delay(h * 1000);
}

////////////   FUNÇÕES /////////////////

void IR_Init() {
    pinMode(IR5, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR1, INPUT);
}

void Motors_Init() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
}

int black_white(int IRsensor, int mini, int maxi) {
    int IR_n;
    if (IRsensor > maxi) {
        IR_n = 0;
    } else if (IRsensor < mini) {
        IR_n = 1;
    }

    return IR_n;
}

void MotorsSpeed(int Vel_Esq, int Vel_Dir) {
    MotorEsqSpeed(Vel_Esq);
    MotorDirSpeed(Vel_Dir);
}
//------------------------------Direito--------------
void MotorDirSpeed(int Speed) {
    if (Speed == 0) {
        Serial.print("esq:");
        Serial.print(esq);
        Serial.print("dir:");
        Serial.print(dir);
        digitalWrite(AIN1, 1);
        digitalWrite(AIN2, 1);
    }
    bool forwards = 1;

    if (Speed < 0) {
        Speed = Speed * (-1);
        forwards = 0;
    }
    if (Speed > 255) Speed = 255;

    Speed = 255 - Speed;

    if (forwards) {
        digitalWrite(AIN1, HIGH);
        analogWrite(AIN2, Speed);
    } else {
        analogWrite(AIN1, Speed);
        digitalWrite(AIN2, HIGH);
    }
}
//------------------------------Esquerdo--------------
void MotorEsqSpeed(int Speed) {
    if (Speed == 0) {
        digitalWrite(BIN1, 1);
        digitalWrite(BIN2, 1);
    }
    bool forwards = 1;

    if (Speed < 0) {
        Speed = Speed * (-1);
        forwards = 0;
    }
    if (Speed > 255) Speed = 255;

    Speed = 255 - Speed;
    if (forwards) {
        digitalWrite(BIN1, HIGH);
        analogWrite(BIN2, Speed);
    } else {
        analogWrite(BIN1, Speed);
        digitalWrite(BIN2, HIGH);
    }
}
