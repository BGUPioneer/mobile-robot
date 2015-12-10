
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <ax12.h>
#include <BioloidController.h>
#include <PID_v1.h>

// we always need to create an instance of the bioloid control, usually with baud = 1Mbps.
BioloidController bioloid = BioloidController(1000000);

ros::NodeHandle  nh;

double Setpoint=0;
double Input=0;
double Output=0;
//double P=1;
//double I=0.5;
double P=0.2;
double I=0;
double D=0;
PID myPID(&Input, &Output, &Setpoint,P,I,D, DIRECT);
unsigned long time;
boolean Started=false;
boolean Initialized=false;
boolean IsErrorReceived=false;


//1deg= 1/0.088 servo unit
//1 rad= 180/(0.088*Pi) servo unit = 651.088404302 servo unit
double Rad2Servo= 651.088404302;
double Servo2Rad=0.00153589;

//Deadband 
int DeadBand = 15;

void start_stopCb( const std_msgs::Bool& msg){
  if (msg.data){
    Started=true;
  }
  else
  {
    Started=false;
    PausePan();
  }
}

void error_cmdCb( const std_msgs::Float32& msg){
  double error=msg.data*Rad2Servo;
  if (abs(error) < DeadBand){
    error = 0;
  }
  Input=-error;
  IsErrorReceived = true;
}

ros::Subscriber<std_msgs::Bool> sub_start_stop("Start_Stop_Pan", start_stopCb );
ros::Subscriber<std_msgs::Float32> sub_error_cmd("Pan_Error_Command", error_cmdCb );

std_msgs::Float32 pan_pos_msg;
ros::Publisher pub_pan_pos("Pan_Feedback", &pan_pos_msg);


void setup()
{
  InitServo();
  InitPID();
  nh.getHardware()->setBaud(250000); 
  nh.initNode();
  nh.advertise(pub_pan_pos);
  nh.subscribe(sub_error_cmd);
  nh.subscribe(sub_start_stop);
  time=millis();
  Initialized=true;
}

void loop()
{
  if (Initialized){
    if(Started){
      unsigned long elapsed=millis()-time;
      if ((elapsed>300)&(!IsErrorReceived)){
        time = millis();
        PausePan();
      }
      if (IsErrorReceived)
      {
        IsErrorReceived = false;
        myPID.SetMode(AUTOMATIC);
        myPID.Compute();
        SetSpeedPM(Output);
        time = millis();
      }
    }
    GetAndPubPanPos();
  }
  nh.spinOnce();
  delay(5);
}

void GetAndPubPanPos()
{
  int PanPos=GetPosition(1);
  pan_pos_msg.data= (float) (PanPos*Servo2Rad - 3.141593);
  pub_pan_pos.publish( &pan_pos_msg );
}

void PausePan()
{
  myPID.SetMode(MANUAL);
  Output = 0;
  SetSpeedPM(Output);
}

void InitServo()
{
  delay(500);
  ax12SetRegister2(1, AX_CW_ANGLE_LIMIT_L, 0); 
  ax12SetRegister2(1, AX_CCW_ANGLE_LIMIT_L, 2048);
  ax12SetRegister2(1, AX_GOAL_SPEED_L , 100);
  delay(500);
  SetPosition(1,2048);
  delay(1000);
  ax12SetRegister2(1, AX_GOAL_SPEED_L , 0);
  ax12SetRegister2(1, AX_CW_ANGLE_LIMIT_L, 0); 
  ax12SetRegister2(1, AX_CCW_ANGLE_LIMIT_L, 0);

}

void InitPID()
{
  myPID.SetOutputLimits(-1023,1023);
  myPID.SetMode(AUTOMATIC);
}

void SetSpeedPM(int speed1023) //Set speed of servo ID, -1023 to 0 -> CW, 0 to 1023 -> CCW
{
  if(abs(speed1023)<1024){
    if(speed1023>0){
      ax12SetRegister2(1, AX_GOAL_SPEED_L , speed1023);
    }
    else{
      ax12SetRegister2(1, AX_GOAL_SPEED_L , abs(speed1023)+1024);
    }
  }
}


