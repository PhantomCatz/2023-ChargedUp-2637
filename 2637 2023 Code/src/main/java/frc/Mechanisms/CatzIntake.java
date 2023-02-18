package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class CatzIntake {
  public CatzIntake Intake;
  private Thread intakeThread;
  //Ritio need correct
  final double INTAKE_MOTOR_GEAR_RATIO = 1.0/49.0;

//-----------------------------------------------------------------------------------------------------------
//Roller
//-----------------------------------------------------------------------------------------------------------
  private CANSparkMax intakeRoller;
  private final int INTAKE_ROLLER_MC_ID        = 30; 

  private final double INTAKE_ROLLER_MOTOR_POWER       = 0.78;;
  private final double OUTTAKE_ROLLER_MOTOR_POWER      = 1.0;
  public  final double AUTON_INTAKE_ROLLER_MOTOR_POWER = 0.8;
  private final double INTAKE_MOTOR_POWER_OFF          = 0.0;

 
  public boolean intakeRollerStartIn = false;
  public boolean intakeRollerOut = false;
 
//-----------------------------------------------------------------------------------------------------------
//Pivot
//-----------------------------------------------------------------------------------------------------------
  private WPI_TalonFX deploymentMotor;
  private final int INTAKE_PIVOT_MC_ID        = 32; 

  private static final int INTAKE_STOW_MINIMUM_ANGLE          = 80;
  private static final int INTAKE_STOW_MAXIMUM_ANGLE          = 92;
  private static final int INTAKE_STOW_REDUCE_ANGLE           = 70;
  private static final int INTAKE_STOW_FULLY_STOWED_ANGLE = 88;
  private static final int INTAKE_DEPLOY_MINIMUM_ANGLE        = 45;
  private static final int INTAKE_DEPLOY_REDUCE_ANGLE         = 45;
  private static final int INTAKE_DEPLOY_FULLY_DEPLOYED_ANGLE = 2;
  

  //Pivot Stow Angle
  private static final int INTAKE_STOW_RANGE_ACCEPTABLE_ANGLE = 2;

  //Pivot Motor Power For IntakeControl
  private final double INTAKE_MOTOR_POWER_START_DEPLOY    =  0.4;
  private final double INTAKE_MOTOR_POWER_REDUCE_DEPLOY   =  0.2;
  private final double INTAKE_MOTOR_POWER_ADJUST_DEPLOY   =  0.15;
  private final double INTAKE_MOTOR_POWER_START_STOW      = -0.40;
  private final double INTAKE_MOTOR_POWER_REDUCE_STOW     = -0.25;
  private final double INTAKE_MOTOR_POWER_ADJUST_STOW     =  0.15;
  private final double INTAKE_MOTOR_POWER_STOP     =  0.0;
  //Pivot Motor Power For Method
  private final double DEPLOY_MOTOR_POWER              = 0.8;
  private final double DEPLOY_MOTOR_ADJUST_POWER       = 0.3;

  //Pivot status
  private final int INTAKE_MODE_NULL                = 0;
  public  final int INTAKE_MODE_DEPLOY_START        = 1;
  private final int INTAKE_MODE_DEPLOY_REDUCE_POWER = 2;
  private final int INTAKE_MODE_DEPLOY_HOLD         = 3;
  public  final int INTAKE_MODE_STOW_START          = 4;
  private final int INTAKE_MODE_STOW_REDUCE_POWER   = 5;
  private final int INTAKE_MODE_STOW_HOLD           = 6;

//Pivot IntakeMode initialization
  public int intakeMode = INTAKE_MODE_NULL;

  public boolean intakeStowed = true;
  public boolean intakeDeployed = false;
//---------------------------------------------definition part end--------------------------------------------------------------
  
  public CatzIntake() {
    Intake = new CatzIntake();
    intakeRoller    = new CANSparkMax(INTAKE_ROLLER_MC_ID,MotorType.kBrushed);
    deploymentMotor = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

    intakeRoller.restoreFactoryDefaults();
    deploymentMotor.configFactoryDefault();

    intakeRoller.setIdleMode(IdleMode.kCoast);
    deploymentMotor.setNeutralMode(NeutralMode.Brake);
    

  }

//---------------------------------------------------Roller--------------------------------------------------------
   

  public void intakeRollerStartIn()
    {
        intakeRoller.set(INTAKE_ROLLER_MOTOR_POWER);
        intakeRollerStartIn = true;
    }

    public void intakeRollerOut()
    {
        intakeRoller.set(-OUTTAKE_ROLLER_MOTOR_POWER);
        intakeRollerOut = true;
    }


    public void intakeRollerOff()
    {
        intakeRoller.set(INTAKE_MOTOR_POWER_OFF);
        intakeRollerStartIn = false;
        intakeRollerOut = false;
    }

//------------------------------------------------Pivot--------------------------------------------------------------

    public void deployIn(){
        deploymentMotor.set(ControlMode.PercentOutput, DEPLOY_MOTOR_POWER);
    }


    public void deployOut(){
        deploymentMotor.set(ControlMode.PercentOutput, -DEPLOY_MOTOR_POWER);
    }

    public void deploymentAdjust(){
        deploymentMotor.set(ControlMode.PercentOutput, -DEPLOY_MOTOR_ADJUST_POWER);
    }

    //assume it's 0 degrees horizontally and 90 degrees vertically
    public void intakeControl(){
    
        intakeThread = new Thread(() ->
        { 
    
            while(true)
            {
                double currentAngle = getIntakeDeployPositionDegrees();
                switch(intakeMode)
                {

                    case INTAKE_MODE_DEPLOY_START:
                        currentAngle = getIntakeDeployPositionDegrees();
                        deploymentMotor.set(INTAKE_MOTOR_POWER_START_DEPLOY);
                        if(currentAngle < INTAKE_DEPLOY_REDUCE_ANGLE)
                        {
                            intakeMode = INTAKE_MODE_DEPLOY_REDUCE_POWER;
                        }
                        intakeDeployed = true;
                    break;

                    case INTAKE_MODE_DEPLOY_REDUCE_POWER:
                        currentAngle = getIntakeDeployPositionDegrees();
                            deploymentMotor.set(INTAKE_MOTOR_POWER_REDUCE_DEPLOY);
                            intakeDeployed = true;
                            Intake.intakeRollerStartIn();
                            if(currentAngle <= INTAKE_DEPLOY_FULLY_DEPLOYED_ANGLE){
                                deploymentMotor.set(INTAKE_MOTOR_POWER_OFF);
                                intakeMode = INTAKE_MODE_DEPLOY_HOLD;
                            }
                        
                    break;

                    case INTAKE_MODE_DEPLOY_HOLD:
                        currentAngle = getIntakeDeployPositionDegrees();
                        if(currentAngle > INTAKE_DEPLOY_MINIMUM_ANGLE)
                        {
                        deploymentMotor.set(INTAKE_MOTOR_POWER_ADJUST_DEPLOY);
                        }
                        break;

    
                    case INTAKE_MODE_STOW_START:
                        currentAngle = getIntakeDeployPositionDegrees();
                        Intake.intakeRollerOff();
                        deploymentMotor.set(INTAKE_MOTOR_POWER_START_STOW);
                        if(currentAngle > INTAKE_STOW_REDUCE_ANGLE){
                        intakeMode = INTAKE_MODE_STOW_REDUCE_POWER;
                        }
                    break;

                    case INTAKE_MODE_STOW_REDUCE_POWER:
                        currentAngle = getIntakeDeployPositionDegrees();
                            deploymentMotor.set(INTAKE_MOTOR_POWER_REDUCE_STOW);
                            intakeDeployed = false;
                            if(currentAngle >= INTAKE_STOW_FULLY_STOWED_ANGLE){
                                deploymentMotor.set(INTAKE_MOTOR_POWER_OFF);
                                intakeMode = INTAKE_MODE_STOW_HOLD;
                            }
                        
                    break;

                    case INTAKE_MODE_STOW_HOLD:
                        currentAngle = getIntakeDeployPositionDegrees();
                        double angleDifference = currentAngle-90;//vertical degree/position
                        if(currentAngle < INTAKE_STOW_MINIMUM_ANGLE)
                        {
                            deploymentMotor.set(-INTAKE_MOTOR_POWER_ADJUST_STOW);
                        }
                        else if(currentAngle > INTAKE_STOW_MAXIMUM_ANGLE){
                        deploymentMotor.set(INTAKE_MOTOR_POWER_ADJUST_STOW);
                        }
                        else if(Math.abs(angleDifference)<= INTAKE_STOW_RANGE_ACCEPTABLE_ANGLE){
                            deploymentMotor.set(INTAKE_MOTOR_POWER_STOP);
                        }
                        intakeStowed = true;
                    break;

                    default:
                        deploymentMotor.set(0.0);
                    break;
                }
            }
        });
        intakeThread.start();
    
}

    public double getIntakeDeployPositionDegrees(){
        //rewrite needed
        double deploymentMotorOutput = deploymentMotor.getSelectedSensorPosition();
        System.out.println(deploymentMotorOutput);
        return Math.abs(deploymentMotorOutput * INTAKE_MOTOR_GEAR_RATIO * 360.0);
    }

}