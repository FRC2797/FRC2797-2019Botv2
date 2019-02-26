package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RotateToAngle extends Command {
    
    //Instantializing class variables for the speed, and angle of movement
    private double speed, angle;
    
    //Rotates the robot to the angle 
    public RotateToAngle(double speed, double angle){
        this.speed = speed;
        this.angle = angle;
    }

    protected void initialize(){
        Robot.drivetrain.rotateToAngle(angle, speed);
    }
    protected void execute(){
    }

    //Checks if robot is at desinated angle
    protected boolean isFinished(){
        return Robot.drivetrain.getGyroPID().onTarget();
    }
}