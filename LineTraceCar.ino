#include <DRV8835MotorShield.h>
DRV8835MotorShield motors;

#define KP 37
#define KI 0
#define KD 2.8
#define target 2
#define CarSpeed_L 160
#define CarSpeed_R 160
#define C2 2.4
#define RA 3.6
#define spin 60

static signed long diff_L[2];
static signed long diff_R[2];
static float integral_L;
static float integral_R;
static int flag;
unsigned long time;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int sVal3 = analogRead(A3);
  int sVal4 = analogRead(A4);
  int sVal5 = analogRead(A5);
  float dt,pretime;
  
  float vR = sVal3 * (5.0 / 1023.0);
  float vC = sVal4 * (5.0 / 1023.0);
  float vL = sVal5 * (5.0 / 1023.0);
  if(vL>C2&&vC>C2&&vR<C2)/*bbw*/
  { 
    flag=0;
  }
  else if(vL>C2&&vC<C2&&vR<C2)/*bww*/
  {
    flag=0;
  }
  else if(vL<C2&&vC>C2&&vR>C2)/*wbb*/
  {
    flag=1;
  }
  else if(vL<C2&&vC<C2&&vR>C2)/*wwb*/
  {
    flag=1;
  }  

  dt=(millis()-pretime)/1000;
  pretime=millis();
  
  time=millis();
  Serial.print(time);
  Serial.print("\t");
  Serial.print(vL);
  Serial.print("\t");
  Serial.print(vC);
  Serial.print("\t");
  Serial.print(vR);
  Serial.print("\n");
  
  int LSPEED=L_PD(vR,dt);
  int RSPEED=R_PD(vL,dt);

  int ResultSpeed_L=CarSpeed_L+LSPEED;
  int ResultSpeed_R=CarSpeed_R+RSPEED;

  if(vL<C2&&vC<C2&&vR<C2)
  {
    if(flag==0)
    {
      motors.setM1Speed(-spin);
      motors.setM2Speed(spin);
    }
    else if(flag==1)
    {
      motors.setM1Speed(spin);
      motors.setM2Speed(-spin);
    } 
  }
  else if(vC>RA&&vR>RA)
    {
      motors.setM1Speed(spin);
      motors.setM2Speed(-spin);
    }
  else if(vL>RA&&vC>RA)
    {
      motors.setM1Speed(-spin);
      motors.setM2Speed(spin);
    }
  else{
      motors.setM1Speed(ResultSpeed_L);
      motors.setM2Speed(ResultSpeed_R);
  }
}

/*左のセンサによるPID*/
float L_PD(float vL,float dt){
  float p,i,d;
 
  diff_L[0]=diff_L[1];
  diff_L[1]=vL-target;
  integral_L+=(diff_L[1]+diff_L[0])/2*dt;
  
  p=KP*diff_L[1];
  i=KI*integral_L;
  d=KD*(diff_L[1]-diff_L[0])/dt;

  return  math_limit(p+i+d);
}

/*右のセンサによるPID*/
float R_PD(float vR,float dt){
  float p,i,d;
 
  diff_R[0]=diff_R[1];
  diff_R[1]=vR-target;
  integral_R+=(diff_R[1]+diff_R[0])/2*dt;
  
  p=KP*diff_R[1];
  i=KI*integral_R;
  d=KD*(diff_R[1]-diff_R[0])/dt;
  
  return math_limit(p+i+d);
}

float math_limit(float pid) {
  pid = constrain(pid, -35 , 150); 

  return pid;
}
