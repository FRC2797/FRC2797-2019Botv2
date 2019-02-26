package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RetractLandingGear extends Command{

    public void execute(){
        Robot.landingGear.retract();
    }

    protected boolean isFinished(){
        return true; 
    }

}
