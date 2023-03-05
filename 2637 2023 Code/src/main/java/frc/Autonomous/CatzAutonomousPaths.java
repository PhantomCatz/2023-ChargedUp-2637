package frc.Autonomous;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

/***************************************************************************
    *
    * Autonomous selections
    * 
***************************************************************************/
public class CatzAutonomousPaths
{  
    private final SendableChooser<Boolean> chosenAllianceColor = new SendableChooser<>();
    private final SendableChooser<Integer> chosenPath = new SendableChooser<>();
    private final SendableChooser<Boolean> chosenBalance = new SendableChooser<>();

    private final double STRAIGHT = 0.0;
    public static double RIGHT = 90.0; 
    public static double LEFT = -90.0;

    private final double TEMP_MAX_TIME = 8.0;
    private final double TEMP_DECEL_DIST = 0.05;
    private final double MIN_DIST = 20.0;
    private final double MAX_DIST = 244.0;
    private final double GP_TO_GP = 48.0;

    public static boolean foward = false;
    public static boolean backward = false;

    public static boolean right = false;
    public static boolean left = false;
    public static boolean all = false; //set all direction sensors to false
    public static boolean pathing = false;
    public static boolean doBalance = true; // default
    
    //MAX SPEED
    private final double FAST = 0.35;
    private final double SLOW = 0.25;

    public static String path;

            
    private final int CENTER_PRELOAD_TAXI_BALANCE = 1;
    private final int SIDE_PRELOAD_INTAKE_SCORE = 2;
    private final int SIDE_2_PRELOAD_INTAKE_SCORE_BALANCE = 3;
    private final int DEFENSE_PRELOAD_POSITIONING = 4;
    private final int CENTER_RIGHT_TUNNEL = 5;
    private final int MANUAL_STICK_MOVEMENT = 6;
    private final int FAR_LEFT_BOOMERANG_CENTER_LEFT = 7;
    private final int FAR_RIGHT_BOOMERANG_CENTER_RIGHT = 8;

     /*  DRIVE STRAIGHT VALUES: 
     * if distance > 70, then FAST, else SLOW
     * 8 second maxTime is an arbitrary number, subject to change upon more testing 
     * only robot backwards movement has negative signs over distance and maxspeed
     * left and right distances and max speed aren't negative
     * TEMP_DECEL_DIST decelDistance is an arbitrary number, subject to change upon more testing
     * 
     *   *note* - autonomous is 15 seconds, meaning that all of this will have to finsih within that time
     *          - THIS CODE IS MADE FOR BLUE SIDE 
     *          - FOR RED, CHANGE LEFTS WITH RIGHTS AND RIGHTS WITH LEFTS (from blue)
     *          - movement similar to code.org level programming
    */

    /* PATH NAME:
     *    /CenterRightTunnel/
     * - CenterRight (Starting Position)
     * - Tunnel (type of movement/movement path)
     */

    /* Distances:          -______-
     * drive.DriveStraight(distance, decelDistance, maxSpeed, wheelPos, maxTime);
     *  - 224 = distance from grid to center pieces
     *                
     */
    // drive.DriveStraight(distance, decelDist, )
    public CatzAutonomousPaths() 
    {
        /*if(all = true) 
        {
            foward = false;
            backward = false;
            right = false;
            left = false;
        } 
        if(Robot.auton.getWheelPos() == 0.0 && Robot.auton.getDistance() > 0) 
        {
            all = true;
            foward = true;
        }
        else if (Robot.auton.getWheelPos() == 0.0 && Robot.auton.getDistance() < 0) 
        {
            all = true;
            backward = true;
        } 
        else if (Robot.auton.getWheelPos() == 90.0) 
        {
            all = true;
            left = true;
        } 
        else if (Robot.auton.getWheelPos() == -90.0) 
        {
            all = true;
            right = true;
        }
        else 
        {
            all = true;
        }*/
    }


    public void initializePathOptions()
    {
        chosenAllianceColor.setDefaultOption("Blue Alliance", Robot.constants.BLUE_ALLIANCE);
        chosenAllianceColor.addOption("Red Alliance", Robot.constants.RED_ALLIANCE);
        SmartDashboard.putData(Robot.constants.ALLIANCE_COLOR, chosenAllianceColor);

        chosenPath.setDefaultOption(Robot.constants.POSITION_SELECTOR1, 1);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR2, 2);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR3, 3);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR4, 4);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR5, 5);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR6, 6);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR7, 7);
        chosenPath.addOption(Robot.constants.POSITION_SELECTOR8, 8);
        SmartDashboard.putData(Robot.constants.ALLIANCE_POSITION, chosenPath);

        chosenBalance.setDefaultOption("Do Balance", Robot.constants.YESBAL);
        chosenBalance.addOption("Don't Balance", Robot.constants.NOOBAL);
        SmartDashboard.putData(Robot.constants.BALANCE, chosenBalance);
    }

    public void Red() 
    {
        RIGHT *= -1;
        LEFT *= -1;
    }

    // DriveStright(distance, decel distance, maxSpeed, steering wheel direction, max run time)
    /**
     * Drive Foward
     */
    public void centerToGrid() 
    {
        Robot.auton.DriveStraight(-MAX_DIST, TEMP_DECEL_DIST, -FAST, STRAIGHT, TEMP_MAX_TIME);
    }
    public void gridToCenter() 
    {
        Robot.auton.DriveStraight(MAX_DIST, TEMP_DECEL_DIST, FAST, STRAIGHT, TEMP_MAX_TIME);
    }
    
    public void gridToAreaInfrontOfCargo() 
    {
        Robot.auton.DriveStraight(MAX_DIST - MIN_DIST, TEMP_DECEL_DIST, FAST, STRAIGHT, TEMP_MAX_TIME);
    }
    
    public void fowardToCargo() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
    }
    public void gridToChargingStation() 
    {
        Robot.auton.DriveStraight(GP_TO_GP, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
    }

    /**
     * Drive Backward
     */

    public void centerToAreaInfrontOfDock() 
    {
        Robot.auton.DriveStraight(-MAX_DIST + MIN_DIST, TEMP_DECEL_DIST, -FAST, STRAIGHT, TEMP_MAX_TIME);
    }
    public void dockToGrid() 
    {
        Robot.auton.DriveStraight(-MIN_DIST, TEMP_DECEL_DIST, -SLOW, STRAIGHT, TEMP_MAX_TIME);
    }
    public void centerToChargingStation() 
    {
        Robot.auton.DriveStraight(-130, TEMP_DECEL_DIST, -FAST, STRAIGHT, TEMP_MAX_TIME); //dist used to be -128.75
    }

    /**
     * Drive Right
     */
    public void translateRight48() 
    {
        Robot.auton.DriveStraight(GP_TO_GP, TEMP_DECEL_DIST, SLOW, RIGHT, TEMP_MAX_TIME);
    }
    public void translateFowardRight() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, RIGHT, TEMP_MAX_TIME);
    }
    public void translateBackRight() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, RIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(-MIN_DIST, TEMP_DECEL_DIST, -SLOW, STRAIGHT, TEMP_MAX_TIME);
    }

    /**
     * Drive Left
     */
    public void translateLeft48() 
    {
        Robot.auton.DriveStraight(GP_TO_GP, TEMP_DECEL_DIST, SLOW, LEFT, TEMP_MAX_TIME);
    }
    public void translateFowardLeft() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, LEFT, TEMP_MAX_TIME);
    }
    public void translateBackLeft() 
    {
        Robot.auton.DriveStraight(MIN_DIST, TEMP_DECEL_DIST, SLOW, LEFT, TEMP_MAX_TIME);
        Robot.auton.DriveStraight(-MIN_DIST, TEMP_DECEL_DIST, -SLOW, STRAIGHT, TEMP_MAX_TIME);
    }    
    public void diagonal()
    {
        Robot.auton.DriveStraight(GP_TO_GP, TEMP_DECEL_DIST, FAST, GP_TO_GP, TEMP_MAX_TIME); //wheelPos used to be 45 (foward left)
        Robot.auton.DriveStraight(0, TEMP_DECEL_DIST, SLOW, STRAIGHT, TEMP_MAX_TIME);
    }

    
    
    public void determinePath()
    {
        if(chosenAllianceColor.getSelected() == Robot.constants.BLUE_ALLIANCE)
        {
            if(chosenPath.getSelected() == CENTER_PRELOAD_TAXI_BALANCE) 
            {
                CenterPreloadTaxiBalance();
            }
            if(chosenPath.getSelected() == SIDE_PRELOAD_INTAKE_SCORE) 
            {
                SidePreloadIntakeScore();
            }
            if(chosenPath.getSelected() == SIDE_2_PRELOAD_INTAKE_SCORE_BALANCE)
            {
                Side2PreloadIntakeScoreBalance();
            }
            if(chosenPath.getSelected() == DEFENSE_PRELOAD_POSITIONING) 
            {
                DefensePreloadPositioning();
            }
            if(chosenPath.getSelected() == CENTER_RIGHT_TUNNEL) 
            {
                centerRightTunnel();
            }
            if(chosenPath.getSelected() == MANUAL_STICK_MOVEMENT) 
            {
                manualStickMovement();
            }
            if(chosenPath.getSelected() == FAR_LEFT_BOOMERANG_CENTER_LEFT) 
            {
                farLeftBoomerangCenterLeft();
            }
            if(chosenPath.getSelected() == FAR_RIGHT_BOOMERANG_CENTER_RIGHT) 
            {
                farRightBoomerangCenterRight();
            }
        }
        else //added "red()" infront of these all to invert all rights and lefts for the red side of the field
        {
            if(chosenPath.getSelected() == CENTER_PRELOAD_TAXI_BALANCE) 
            {
                Red(); CenterPreloadTaxiBalance();
            }
            if(chosenPath.getSelected() == SIDE_PRELOAD_INTAKE_SCORE) 
            {
                Red(); SidePreloadIntakeScore();
            }
            if(chosenPath.getSelected() == SIDE_2_PRELOAD_INTAKE_SCORE_BALANCE) 
            {
                Red(); Side2PreloadIntakeScoreBalance();
            }
            if(chosenPath.getSelected() == DEFENSE_PRELOAD_POSITIONING) 
            {
                Red(); DefensePreloadPositioning();
            }
            if(chosenPath.getSelected() == CENTER_RIGHT_TUNNEL) 
            {
                Red(); centerRightTunnel();
            }
            if(chosenPath.getSelected() == MANUAL_STICK_MOVEMENT)
            {
                Red(); manualStickMovement();
            }
            if(chosenPath.getSelected() == FAR_LEFT_BOOMERANG_CENTER_LEFT) 
            {
                Red(); farLeftBoomerangCenterLeft();
            }
            if(chosenPath.getSelected() == FAR_RIGHT_BOOMERANG_CENTER_RIGHT) 
            {
                Red(); farRightBoomerangCenterRight();
            }
           
        }
    }

    /********************************** */
    public void CenterPreloadTaxiBalance() 
    {
        pathing = true;
        path = "Center Left Tunnel";
        dockToGrid();
        //score code
        gridToCenter();
        //pickup cone;
        if(doBalance = true) 
        {
            centerToChargingStation();
            Robot.balance.StartBalancing();
        }

        Robot.auton.StopDriving();
        pathing = false;
    }
    
    /********************************** */
    public void SidePreloadIntakeScore()
    {
        pathing = true;
        path = "Center Left Tunnel";
        dockToGrid();
        //score code
        gridToCenter();
        //pickup cone;
        centerToGrid();

        Robot.auton.StopDriving();
        pathing = false;
    }

    /***************************************** */
    public void Side2PreloadIntakeScoreBalance() 
    {
        pathing = true;
        path = "..";
        translateBackLeft();
        //score cone
        translateFowardRight();
        gridToCenter();
        //pickup cube
        centerToGrid();
        //score cube
        translateFowardRight();
        translateRight48();
        if(doBalance = true) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }

        Robot.auton.StopDriving();
        pathing = false;
    }

    /********************************** */
    public void DefensePreloadPositioning() 
    {
        pathing = true;
        path = "Far Left Exit Community";
        translateBackLeft();
        //score cone;
        translateFowardRight();
        gridToAreaInfrontOfCargo();
        diagonal();

        Robot.auton.StopDriving();
        pathing = false;
    }


    public void centerRightTunnel() 
    {  
        pathing = true;
        path = "Center Right Tunnel";
        //values might have to be changed since we are going over the charging station
        dockToGrid();
        //score cone;
        gridToCenter();
        //pickup cone;
        centerToAreaInfrontOfDock();
        translateBackRight();
        //score cone;
        translateFowardLeft();
        if(doBalance = true) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }
        Robot.auton.StopDriving();
        pathing = false;
        
    }

    public void manualStickMovement() 
    {
        pathing = true;
        path = "Manual Car Stick";
        dockToGrid();
        //score cone;
        gridToCenter();
        //pickup cone;
        centerToAreaInfrontOfDock();
        translateBackRight();
        //score cone
        translateFowardLeft();
        gridToAreaInfrontOfCargo();
        //pickup cone
        centerToGrid();
        //score cone
        if(doBalance = true) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }

        Robot.auton.StopDriving();
        pathing = false;
    }


    public void farLeftBoomerangCenterLeft()
    {
        pathing = true;
        path = "Far Left Boomerang Center Left";
        translateBackLeft();
        //score cone;
        translateFowardRight();
        gridToAreaInfrontOfCargo();
        //pickup cube
        centerToGrid();
        //score cube
        gridToAreaInfrontOfCargo();
        translateRight48();
        fowardToCargo();
        //pickup cone
        centerToGrid();
        //score cone
        if(doBalance = true) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }

        Robot.auton.StopDriving();
        pathing = false;
    }


    public void farRightBoomerangCenterRight()
    {
        pathing = true;
        path = "Far Right Boomerang Center Right";
        translateBackRight();
        //score cone;
        translateFowardLeft();
        gridToAreaInfrontOfCargo();
        //pickup cube
        centerToGrid();
        //score cube
        gridToAreaInfrontOfCargo();
        translateLeft48();
        fowardToCargo();
        //pickup cone
        centerToGrid();
        //score cone
        if(doBalance = true) 
        {
            gridToChargingStation();
            Robot.balance.StartBalancing();
        }

        Robot.auton.StopDriving();
        pathing = false;
    }
    


    public static void updateShuffleboard()
    {
        SmartDashboard.putBoolean("Foward", foward);
        SmartDashboard.putBoolean("Backward", backward);
        SmartDashboard.putBoolean("Left", left);
        SmartDashboard.putBoolean("Right", right);
    }

}

