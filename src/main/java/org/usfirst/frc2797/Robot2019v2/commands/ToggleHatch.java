package org.usfirst.frc2797.Robot2019v2.commands;
import org.usfirst.frc2797.Robot2019v2.Robot;
import edu.wpi.first.wpilibj.command.Command;

//Extends or retracts the hatch
public class ToggleHatch extends Command{

    protected void execute(){
        Robot.hatch.toggle();
    }

    //Checks if hatch is extended or retracted
    protected boolean isFinished(){
        return true;
    }

}