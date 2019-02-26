package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;
import edu.wpi.first.wpilibj.command.Command;

//Brings hatch back
public class RetractHatch extends Command{
    public void execute(){
        Robot.hatch.retract();
    }

    //Checks if hatch is retracted
    protected boolean isFinished(){
        return true;
    }

}