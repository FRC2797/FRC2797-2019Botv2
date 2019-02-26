package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;
import edu.wpi.first.wpilibj.command.Command;

//Drops hatch off robot
public class ExtendHatch extends Command{

    public void execute(){
        Robot.hatch.extend();
    }

    //Checks if hatchs is extended
    protected boolean isFinished(){
        return true;
    }

}