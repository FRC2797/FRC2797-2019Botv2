package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

//Retracts the arm back
public class RetractArm extends Command{
    public void execute(){
        Robot.arm.retract();
        
    }

    //Checks if arm is retracted 
    protected boolean isFinished(){
        return true;
    }
}