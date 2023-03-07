package frc.Mechanisms;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.Robot;

public class CatzRGB 
{
    private final int LED_COUNT = 54;
    private final int COLOR1_LED_COUNT = LED_COUNT / 2;

    private final int MAX_LED_BUFFER_INDEX = LED_COUNT - 1;

    private final int LED_PWM_PORT_FRNT = 0;
    private final int LED_PWM_PORT_BACK = 3;

    private AddressableLED ledFrnt;
    //private AddressableLED ledBack;

    private AddressableLEDBuffer ledBufferFrnt;
    //private AddressableLEDBuffer ledBufferBack;
    
    private Color RED    = Color.kRed;
    private Color ORANGE = Color.kOrange;
    private Color YELLOW = Color.kYellow;
    private Color GREEN  = Color.kGreen;
    private Color BLUE   = Color.kBlue;
    private Color PURPLE = Color.kPurple;
    private Color WHITE  = Color.kWhite;
    private Color TEAM_COLOR = Color.kDodgerBlue;
    private Color PINK  = Color.kHotPink;

    private Color color1;
    private Color color2;

    private int ledIndex = 0;
    private int flowArrayOffset = 0;
    private int arrayIndex = 0;

    private int ledDelay = 0;
    private final int FLOW_UP_DELAY_COUNT = 2;
    private final int FLOW_DN_DELAY_COUNT = 2;
    private final int FLASH_DELAY_COUNT   = 15;

    private int nextFlashColor = 1;

    private ArrayList<Color> ledPattern;

    private final boolean LED_ON = true;
    private final boolean LED_OFF = true;
    private boolean LED_FRNT = LED_OFF;

    


    public CatzRGB()
    {
        ledFrnt = new AddressableLED(LED_PWM_PORT_FRNT);
        //ledBack = new AddressableLED(LED_PWM_PORT_BACK);

        ledBufferFrnt = new AddressableLEDBuffer(LED_COUNT);
       // ledBufferBack = new AddressableLEDBuffer(LED_COUNT);

        ledFrnt.setLength(ledBufferFrnt.getLength());
        ledFrnt.setData(ledBufferFrnt);
        ledFrnt.start();

       // ledBack.setLength(ledBufferBack.getLength());
        //ledBack.setData(ledBufferBack);
        //ledBack.start();

        ledPattern = new ArrayList<Color>(LED_COUNT);

        for(int i = 0; i < LED_COUNT; i++)
        {
            ledPattern.add(i, TEAM_COLOR);
        }
    }

    public void setLEDPattern(Color color1, Color color2)
    {
        for(int i = 0; i < COLOR1_LED_COUNT; i++)
        {
            ledPattern.set(i, color1);
        }
        for(int i = COLOR1_LED_COUNT; i < LED_COUNT; i++)
        {
            ledPattern.set(i, color2);
        }
    }

    public void setLEDPattern(Color color)
    {
        for(int i = 0; i < LED_COUNT; i++)
        {
            ledPattern.set(i, color);
        }
    }

    public void LEDWork()
    {
        if(Robot.isInDisabled() == true)
        {
            solidColor(GREEN);
           // need to add pingpong team color
        }
        else if(Robot.isInAuton() == true)
        {
            if(Robot.balance.startBalance == true)
            {
                solidColor(GREEN);
            }
            else
            {
                setLEDPattern(TEAM_COLOR, WHITE);
                if(ledDelay > FLOW_UP_DELAY_COUNT)
                {
                    flowUp();
                    ledDelay = 0;
                }
            }
        }
        else if(Robot.indexer.isConeDetected == true)
        {
            solidColor(PURPLE);
        }
        else if(Robot.indexer.isConeDetected == true)
        {
            solidColor(YELLOW);
        }
        else if(Robot.cubeScoringReady == true)
        {
            setLEDPattern(PURPLE, WHITE);
            if(ledDelay > FLOW_DN_DELAY_COUNT)
            {
                flowDown();
                ledDelay = 0;
            }
        }
        else if(Robot.coneScoringReady == true)
        {
            setLEDPattern(YELLOW, WHITE);
            if(ledDelay > FLOW_DN_DELAY_COUNT)
            {
                flowDown();
                ledDelay = 0;
            }
        }
        else if(Robot.noGamePiece == true)
        {
            setLEDPattern(TEAM_COLOR, WHITE);
            if(ledDelay > FLOW_DN_DELAY_COUNT)
            {
                flowDown();
                ledDelay = 0;
            }
        }
        
        //solidColor(GREEN);
        ledDelay++;
        ledFrnt.setData(ledBufferFrnt);
       /*  if(checkGyro())
        {
            ledFrnt.setData(ledBufferFrnt);
        }
        else
        {
            ledFrnt.setData(ledBufferFrnt);
            //ledBack.setData(ledBufferBack);
        }   */  
        
        
    }
    
    public void flash(Color color1, Color color2)
    {
        if(ledDelay > FLASH_DELAY_COUNT)
        {
            if(nextFlashColor == 1)
            {
                solidColor(color1);
                nextFlashColor = 2;
            }
            else
            {
                solidColor(color2);
                nextFlashColor = 1;
            }

            ledDelay = 0;
        }
    }

    public void solidColor(Color color)
    {
        for(int i = 0; i < LED_COUNT; i++)
        {
            if(checkGyro())
            {
                ledBufferFrnt.setLED(i, color);
            }
            else
            {
                //ledBufferBack.setLED(i, color);
            }
        }
    }

    public void flowDown()
    {
        for(ledIndex = 0; ledIndex < LED_COUNT; ledIndex++)
        {
            arrayIndex = (flowArrayOffset + ledIndex) % LED_COUNT;
            if(checkGyro())
            {
                ledBufferFrnt.setLED(ledIndex, ledPattern.get(arrayIndex));
            }
            else 
            {
                //ledBufferBack.setLED(ledIndex, ledPattern.get(arrayIndex));
            }
            
        }
        
        flowArrayOffset++;

        if(flowArrayOffset >= LED_COUNT)
        {
            flowArrayOffset = 0;
        }
    }

    public void flowUp()
    {
        for(ledIndex = 0; ledIndex < LED_COUNT; ledIndex++)
        {
            arrayIndex = (flowArrayOffset + ledIndex) % LED_COUNT;
            if(checkGyro())
            {
                ledBufferFrnt.setLED(ledIndex, ledPattern.get(arrayIndex));
            }
            else
            {
               // ledBufferBack.setLED(ledIndex, ledPattern.get(arrayIndex));
            }
           
        }
        
        flowArrayOffset--;

        if(flowArrayOffset < 0)
        {
            flowArrayOffset = MAX_LED_BUFFER_INDEX;
        }
    }

    public boolean checkGyro()
    {
        if(Math.abs(Robot.drivetrain.getGyroAngle()) <90 )
        {
            LED_FRNT = LED_OFF;
        }
        else
        {
            LED_FRNT = LED_ON;
        }
        return LED_FRNT;
    }
}