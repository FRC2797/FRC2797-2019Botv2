package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

//Extends or retracts arm 
public class ToggleArm extends Command{

    public void execute(){
        Robot.arm.toggle();
    }

    //Checks if arm is extended or retracted
    protected boolean isFinished(){
        return true;
    }
}