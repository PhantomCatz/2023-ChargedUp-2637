package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;

 
public class CatzIntake {

  private Thread intakeThread;
  
//-----------------------------------------------------------------------------------------------------------
//Roller
//-----------------------------------------------------------------------------------------------------------
  //private  VictorSPX intakeRoller;
  private WPI_TalonFX intakeRollerMotor;
  //correction needed
  private final int INTAKE_ROLLER_MC_ID        = 11; 

  private final double INTAKE_ROLLER_MOTOR_POWER       = 0.5;
  private final double OUTTAKE_ROLLER_MOTOR_POWER      = 0.5;
  private final double INTAKE_MOTOR_POWER_OFF          = 0.0;

  public final int INTAKE_ROLLER_OFF = 0;
  public final int INTAKE_ROLLER_IN  = 1;
  public final int INTAKE_ROLLER_OUT = 2;
  public final int INTAKE_ROLLER_UNINITIALIZED = -999;
  public int   intakeRollerState = INTAKE_ROLLER_OFF;

//-----------------------------------------------------------------------------------------------------------
//Pivot
//-----------------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakePivotMotor;
  private final int INTAKE_PIVOT_MC_ID        = 10; 

  private final int INTAKE_PIVOT_FULLY_DEPLOYED_ANGLE = 90;

  private final double INTAKE_PIVOT_PINION_GEAR = 11.0;
  private final double INTAKE_PIVOT_SPUR_GEAR   = 56.0;  
  private final double INTAKE_PIVOT_GEAR_RATIO  =INTAKE_PIVOT_SPUR_GEAR/INTAKE_PIVOT_PINION_GEAR;

  private final double INTAKE_PIVOT_SPROCKET_1  = 16.0;
  private final double INTAKE_PIVOT_SPROCKET_2  = 56.0;
  private final double INTAKE_PIVOT_SPROCKET_RATIO  = INTAKE_PIVOT_SPROCKET_2/INTAKE_PIVOT_SPROCKET_1;
  
  private final double INTAKE_PIVOT_FINAL_RATIO = INTAKE_PIVOT_GEAR_RATIO*INTAKE_PIVOT_SPROCKET_RATIO;




  //Pivot status
  public  final int INTAKE_MODE_NULL                = 0;
  public  final int INTAKE_MODE_DEPLOY              = 1;
  public  final int INTAKE_MODE_INITIALIZATION      = 2;
  public  final int INTAKE_MODE_STOW                = 3;
  public  final int INTAKE_MODE_STOW_HOLD           = 4;

//Pivot IntakeMode initialization
  public int intakeMode = INTAKE_MODE_NULL;

  public boolean intakeStowed = true;
  public boolean intakeDeployed = false;

  public int counter=0;

  public final int INTAKE_PIVOT_STOWED = 0;
  public final int INTAKE_PIVOT_DEPLOYED = 1;
  public final int INTAKE_PIVOT_IN_TRANSIT = 2;
  public final int INTAKE_PIVOT_UNINITIALIZED = -999;
  public int intakePivotState = INTAKE_PIVOT_STOWED;

  public Timer pivotTimer;

  public static double time=Timer.getFPGATimestamp();

  public static double finalMotorPower = 0;
  public static double Kp = 0.01;
  public static double Kd = 0.001;
  public final int INTAKE_DEPLOY_FINAL_ANGLE = 90;
  public final int INTAKE_DEPLOY_INITIAL_ANGLE = 0;
  public final int INTAKE_STOW_FINAL_ANGLE = 0;
  public final int INTAKE_STOW_INITIAL_ANGLE = 90;
  public final double INTAKE_DEPLOYMENT_TIME = 0.26;
  public static double Ot=0;
  public static double OtDot = 0;
  public static double deltaO = 0;

  public static double a3;
  public static double a4;
  public static double a5;
  public static double angleDot = 0;
  public static double angleOld = 0;
  public static double currentAngle = 0;
  public static double timeOld = 0;
  public static double deltaTime = 0;
  public static double b=0;

  public static double powerForMotor = 5.24 * (time *time *time) - 30.24 *(time *time *time*time) + 46.52 *(time *time *time *time *time);


//---------------------------------------------definition part end--------------------------------------------------------------
  
  public CatzIntake() {
    //need add softlimits

    intakeRollerMotor    = new WPI_TalonFX(INTAKE_ROLLER_MC_ID);
    intakePivotMotor = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

    intakeRollerMotor.configFactoryDefault();
    intakePivotMotor.configFactoryDefault();

    // intakeRoller.setSomething?current limit
    intakeRollerMotor.setNeutralMode(NeutralMode.Coast);

    //Timer pivotTimer = new Timer();

   
   

   //initialize for pivot motor
   //sensor-->0;
    intakePivotMotor.setNeutralMode(NeutralMode.Brake);

  

  }


//---------------------------------------------------Roller--------------------------------------------------------
   

  public void intakeRollerIn()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,-INTAKE_ROLLER_MOTOR_POWER);
        intakeRollerState = INTAKE_ROLLER_IN;
    }

    public void intakeRollerOut()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,OUTTAKE_ROLLER_MOTOR_POWER);
        intakeRollerState = INTAKE_ROLLER_OUT;
    }


    public void intakeRollerOff()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_OFF);
        intakeRollerState = INTAKE_ROLLER_OFF;
    }


//------------------------------------------------Pivot--------------------------------------------------------------
    
    public void intakeControl(){
    
        intakeThread = new Thread(() ->
        { 
            while(true)
            {
                //int currentAngle = (int)getIntakeDeployPositionDegrees();
                switch(intakeMode)
                {
                    case INTAKE_MODE_DEPLOY:
                        if(counter ==0 ){
                        pivotTimer.reset();
                        }
                        
                        counter++;
                        
                        currentAngle = getIntakeDeployPositionDegrees();
                        time = pivotTimer.get();

                        deltaO = currentAngle - angleOld;
                        deltaTime = time - timeOld;
                        angleOld = getIntakeDeployPositionDegrees();
                        timeOld = pivotTimer.get();

                        angleDot = deltaO/deltaTime;
                        
                        
                        b = (INTAKE_DEPLOY_FINAL_ANGLE-INTAKE_DEPLOY_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;
                       
                        powerForMotor = 5.24 * (time *time *time) - 30.24 *(time *time *time*time) + 46.52 *(time *time *time *time *time);
                        a3 = 10*b/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
                        a4 = -15*b/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
                        a5 = 6*b/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
                        Ot = (a3*time*time*time) + (a4*time*time*time*time) + (a5*time*time*time*time*time);
                        OtDot = (3 * a3 * time * time) + (4 * a4 * time *time * time) + (5 * a5 * time * time * time * time);
                        finalMotorPower = powerForMotor + Kp*(Ot - getIntakeDeployPositionDegrees()) + Kd*(OtDot - deltaO); 
                        intakePivotMotor.set(finalMotorPower);
                        
                        break;

                    case INTAKE_MODE_STOW:
                        
                    if(counter ==0 ){
                        pivotTimer.reset();
                        }
                        
                        counter++;
                        
                        currentAngle = getIntakeDeployPositionDegrees();
                        time = pivotTimer.get();

                        deltaO = currentAngle - angleOld;
                        deltaTime = time - timeOld;
                        angleOld = getIntakeDeployPositionDegrees();
                        timeOld = pivotTimer.get();

                        angleDot = deltaO/deltaTime;
                        
                        
                        b = (INTAKE_STOW_FINAL_ANGLE-INTAKE_STOW_INITIAL_ANGLE)/INTAKE_DEPLOYMENT_TIME;//time need correct
                       
                        powerForMotor = 5.24 * (time *time *time) - 30.24 *(time *time *time*time) + 46.52 *(time *time *time *time *time);
                        a3 = 10*b/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
                        a4 = -15*b/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
                        a5 = 6*b/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME/INTAKE_DEPLOYMENT_TIME;
                        Ot = (a3*time*time*time) + (a4*time*time*time*time) + (a5*time*time*time*time*time);
                        OtDot = (3 * a3 * time * time) + (4 * a4 * time *time * time) + (5 * a5 * time * time * time * time);
                        finalMotorPower = powerForMotor + Kp*(Ot - getIntakeDeployPositionDegrees()) + Kd*(OtDot - deltaO); 
                        intakePivotMotor.set(finalMotorPower);
                       
                    break;


                    case INTAKE_MODE_STOW_HOLD:
                        
                       
                    break;

                    default:
                        intakePivotMotor.set(INTAKE_MOTOR_POWER_OFF);
                        intakeRollerOff();
                    break;
                }
                
            }
        });
        intakeThread.start();
    
}



    public double getIntakeDeployPositionDegrees(){
        double deploymentMotorRawPosition = intakePivotMotor.getSelectedSensorPosition();
        return ((deploymentMotorRawPosition / 2048) *360 * INTAKE_PIVOT_FINAL_RATIO ) % 360;//deploymentMotorRawPosition 0-4096 is 1 rotation
        //% count will goes
        //
    }

  
}