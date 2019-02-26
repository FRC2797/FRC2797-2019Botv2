package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

//Extends the Arm
public class ExtendArm extends Command{
    
    public void execute(){
        Robot.arm.extend();
    }

    //Checks if the arm is extended 
    protected boolean isFinished(){
        return true;
    }
}