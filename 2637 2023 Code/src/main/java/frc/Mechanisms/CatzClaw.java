package frc.Mechanisms;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class CatzClaw 
{
    
     
    private boolean catzClawOpen = true;
    private boolean catzClawClose = false; 
    public final PneumaticsModuleType PH_TYPE = PneumaticsModuleType.REVPH;
    
    private static DoubleSolenoid clawSolenoid;
    private final int CLAW_EXTEND_PCM_PORT  = 0; // when the pistion is extended the claw closes 
    private final int CLAW_RETRACT_PCM_PORT = 1; // when the pistion is retracted the claw opens 
 
    
    public CatzClaw () 
    {
        clawSolenoid = new DoubleSolenoid(PH_TYPE,CLAW_EXTEND_PCM_PORT,CLAW_RETRACT_PCM_PORT);
    
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