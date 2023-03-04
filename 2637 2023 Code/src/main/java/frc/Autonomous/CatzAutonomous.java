package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.*;

public class CatzAutonomous 
{
    //drive straight variables
    public Boolean startDriving = false;

    final double DRV_S_GEAR_RATIO = 1.0/6.75;
    final double DRV_S_THREAD_PERIOD = 0.02;

    final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV      = 2048.0;
    final double DRVTRAIN_WHEEL_DIAMETER                  = 4.0;
    final double DRVTRAIN_WHEEL_CIRCUMFERENCE             = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);
    final double DRVTRAIN_ENC_COUNTS_TO_INCH              = DRV_S_GEAR_RATIO * DRVTRAIN_WHEEL_CIRCUMFERENCE / TALONFX_INTEGRATED_ENC_CNTS_PER_REV;

    private double DRV_S_STOP_DISTANCE = 0.5;
    private double DRV_S_MIN_POWER = 0.1;
    private double DRV_S_ERROR_GAIN = 0.02;
    private double DRV_S_RATE_GAIN = 0.01;

    private double drvSdistanceOffset = 0.0;
    private double drvSdistanceRemain = 0.0;
    private double drvStargetPower = 0.0;
    private double drvSturnPower = 0.0;
    private double drvSangleOffset = 0.0; //change for the final code EL 2/4
    private double drvScurrentAngle = 0.0;
    private double drvScurrentError = 0.0;
    private double drvSprevError = 0.0;
    private double time = 0.0;
    private double prevTime = -1.0; // no big initial rate
    private double drvSerrorRate = 0;
    private Boolean drvSbackwards;
    public double drvSwheelPos = 0.0;
    public double drvSdistance = 0.0;

    //turn in place variables
    private final static double PID_TURN_THRESHOLD   = 1.25;

	private final static double PID_TURN_IN_PLACE_KP = 0.008;
    
    private final static double TURN_DRIVE_MAX_POS_POWER  =  0.6;
	private final static double TURN_DRIVE_MAX_NEG_POWER  = -0.6;

    private final static double TURN_DRIVE_MIN_POWER = 0.1;

    private final static double TURN_IN_PLACE_PERIOD = 0.010;


	private static double turnCurrentError; 

	private static double turnCurrentAngle;
	private static double turnTargetAngle;

    private static double turnPower;

    private Timer autonTimer;
    public CatzLog data;

    public CatzAutonomous()
    {
        autonTimer = new Timer();
    }

    public void DriveStraight(double drvSdistance, double decelDistance, double maxSpeed,double drvSwheelPos, double maxTime)
    { 
        this.drvSwheelPos = drvSwheelPos;
        this.drvSdistance = drvSdistance;

        startDriving = true;

        if(drvSdistance < 0)
        {
            drvSbackwards = true;
        }
        else
        {
            drvSbackwards = false;
        }

        drvSdistanceOffset = Robot.drivetrain.getAveragePosition();
        drvSangleOffset = Robot.navX.getAngle(); // change for final 2/4 EL
        drvSdistanceRemain = drvSdistance;

        autonTimer.reset();
        autonTimer.start();

        while(Math.abs(drvSdistanceRemain) >= DRV_S_STOP_DISTANCE && startDriving && time < maxTime)
        {
            time = autonTimer.get();
            drvScurrentAngle = Robot.navX.getAngle();
            drvScurrentError = drvSangleOffset - drvScurrentAngle;
            drvSerrorRate = (drvScurrentError - drvSprevError) / (time - prevTime);

            drvSdistanceRemain = drvSdistance + (Robot.drivetrain.getAveragePosition() - drvSdistanceOffset) * DRVTRAIN_ENC_COUNTS_TO_INCH;
            drvStargetPower = -Clamp(-1.0, drvSdistanceRemain / drvSdistance / decelDistance, 1.0) * maxSpeed;
            drvSturnPower = Clamp(-1.0,-DRV_S_ERROR_GAIN * drvScurrentError - DRV_S_RATE_GAIN * drvSerrorRate, 1.0); //"-" in front of Error and Rate
            
            if(Math.abs(drvStargetPower) < DRV_S_MIN_POWER)
            {
                drvStargetPower = DRV_S_MIN_POWER * Math.signum(drvStargetPower);
            }

            if(drvSbackwards)
            {
                drvSturnPower = -drvSturnPower;
            }

            Robot.drivetrain.translateTurn(drvSwheelPos, drvStargetPower, drvSturnPower, Robot.drivetrain.getGyroAngle()); //TBD need to check
            

            prevTime = time;
            drvSprevError = drvScurrentError;

            if(DataCollection.getLogDataID() == DataCollection.LOG_ID_DRV_STRAIGHT)
            {
                data = new CatzLog(Robot.currentTime.get(), drvSdistanceRemain, Robot.drivetrain.getAveragePosition(), drvStargetPower, drvScurrentError, drvScurrentAngle, drvSerrorRate, drvSturnPower, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);  
                Robot.dataCollection.logData.add(data);
            }
            Timer.delay(DRV_S_THREAD_PERIOD);
        }

        //Robot.drivetrain.autoDrive(0); do we need this? 
        Robot.drivetrain.setDrivePower(0);

        startDriving = false;
    }

    public void TurnInPlace(double degreesToTurn, double timeoutSeconds)
    {
        boolean turnInPlaceDone = false;

        double  currentTime       = 0.0;
        double  angleRemainingAbs = 999.0;

        autonTimer.reset();
        autonTimer.start(); 

        turnCurrentAngle  = Robot.navX.getAngle();
        turnTargetAngle   = degreesToTurn + turnCurrentAngle;

        while (turnInPlaceDone == false)
        {
            currentTime  = autonTimer.get();
            turnCurrentAngle = Robot.navX.getAngle();
    
            // calculate error
            turnCurrentError      = turnTargetAngle - turnCurrentAngle;
            angleRemainingAbs = Math.abs(turnCurrentError);

            if (angleRemainingAbs < PID_TURN_THRESHOLD) 
            { 
                turnInPlaceDone = true;
                Robot.drivetrain.rotateInPlace(0.0);
            }
            else
            {
                if (currentTime > timeoutSeconds) 
                {
                    turnInPlaceDone = true;
                    Robot.drivetrain.rotateInPlace(0.0);
                } 
                else
                {
                    turnPower = turnCurrentError * PID_TURN_IN_PLACE_KP;

                    //Clamp
                    //MAX POWER
                    if(turnPower >= TURN_DRIVE_MAX_POS_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_POS_POWER;
                    } 
                    else if(turnPower <= TURN_DRIVE_MAX_NEG_POWER)
                    {
                        turnPower = TURN_DRIVE_MAX_NEG_POWER;
                    }

                    //MIN POWER
                    if (Math.abs(turnPower) < TURN_DRIVE_MIN_POWER)
                    {
                        turnPower = Math.signum(turnPower) * TURN_DRIVE_MIN_POWER;
                    }
                    
                    Robot.drivetrain.rotateInPlace(turnPower);
                }
            }

            if(DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_TURN_IN_PLACE)
            {
            data = new CatzLog(currentTime, turnCurrentAngle, turnCurrentError, turnPower,
                                -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999);

            Robot.dataCollection.logData.add(data);
            }

            Timer.delay(TURN_IN_PLACE_PERIOD);
        }
    }

    public void StopDriving()
    {
        startDriving = false;
    }

    public double Clamp(double min, double in, double max)
    {
        if(in > max)
        {
            return max;
        }
        else if(in < min)
        {
            return min;
        }
        else
        {
            return in;
        }
    }

    public double getWheelPos()
    {
        return drvSwheelPos;
    }

    public double getDistance()
    {
        return drvSdistance;
    }
}
