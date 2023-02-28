package frc.Mechanisms;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class CatzClaw 
{

     
    private boolean catzClawOpen = true;
    private boolean catzClawClose = false; 
    public final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
    public final int PH_CAN_ID = 1;
    public PneumaticHub pneumaticHub = new PneumaticHub(PH_CAN_ID);
    private static DoubleSolenoid clawSolenoid;
    private final int CLAW_EXTEND_PCM_PORT  = 0; // when the pistion is extended the claw closes 
    private final int CLAW_RETRACT_PCM_PORT = 1; // when the pistion is retracted the claw opens 
 
    
    public CatzClaw () 
    {
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,CLAW_EXTEND_PCM_PORT,CLAW_RETRACT_PCM_PORT);
    
    }
    public void catzClawOpen() // claw closes
    {
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
        catzClawClose = false;
    }

    public void catzClawClose() // claw opens
    {
        clawSolenoid.set(DoubleSolenoid.Value.kReverse);
        catzClawOpen = true;
    }

    public edu.wpi.first.wpilibj.DoubleSolenoid.Value getClawState()
    {
        return clawSolenoid.get();
    }

    public void toggle()
    {
        clawSolenoid.toggle();
    }

}