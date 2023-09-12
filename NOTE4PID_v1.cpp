/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;//系统输出值，比如pwm
    myInput = Input; //系统输入值，比如电机或轮胎的转速
    mySetpoint = Setpoint; //这个是预计目标，例如期待电机和轮胎的转速，注意单位要一致
    inAuto = false;//表明当前是否处于自动模式

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to
	//系统的输出范围，arduino默认就是0-255											//the arduino pwm limits

    SampleTime = 100;//采样时间							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);//设置控制方向，例如pwm增加，电机转速增加，这就是DIRECT
    PID::SetTunings(Kp, Ki, Kd, POn);//优化用户提供的系数

    lastTime = millis()-SampleTime;//最近一次采样周期的开始位置？
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 *   注意这个方法，在loop中每次调用。该方法会决定是否一个新的输出需要被计算
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *myInput; //控制系统输入，例如转速 v(t)
      double error = *mySetpoint - input;//输入与期望的误差， error(t)
      double dInput = (input - lastInput);//lastInput=上一次compute时的系统输入 v(t-1)
      outputSum+= (ki * error);//?和公式不一样，不应该历史误差之和么【积分项I，与公式不一致】
      //检查代码可知outputSum从不清零，也是累加和，但公式中是误差之和，不带系数

      /*Add Proportional on Measurement, if P_ON_M is specified*///？测量加入比例  http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
      if(!pOnE) outputSum-= kp * dInput;  //使用Proportional on Measurement【比例项P，这是POM】

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified 给误差加比例系数，如果P_ON_E有效*/
	   double output;//这是真正的输出，要复制给myOutput的！【保存最终结果】
      if(pOnE) output = kp * error;//传统的Proportional on Error【比例项P，和公式一致】
      else output = 0;

      /*Compute Rest of PID Output*/
      output += outputSum - kd * dInput;//差分做了推导，消去了setpoint【差分项D，和公式一致】

	    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	    *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	    return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 * 动态调整控制器的动态性能
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;//小于零不行

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;//用秒来表示的采样周期
   kp = Kp;
   ki = Ki * SampleTimeInSec;//为积分做准备
   kd = Kd / SampleTimeInSec;//为微分做准备

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 *  设置控制系统的输出限制，例如控制电机的 pwm
 *   控制系统的输入限制默认为0-1023（SetInputLimits没找到这个方法）
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 * 设置模式，用来设置控制器的模式，为手动=0，还是自动=非0
 * 当从手动切换到自动时，会初始化PID
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 *该方法为了无缝从手动切换到自动模式
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 * DIRECT模式意味着  Output增加导致Input增加
 * REVERSE 反之，Output增加导致Input减少
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

