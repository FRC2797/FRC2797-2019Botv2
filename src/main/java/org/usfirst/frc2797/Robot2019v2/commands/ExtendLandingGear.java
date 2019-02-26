package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ExtendLandingGear extends Command{
    
    public void execute(){
        Robot.landingGear.extend();
    }

    protected boolean isFinished(){
        return true; 
    }

}
