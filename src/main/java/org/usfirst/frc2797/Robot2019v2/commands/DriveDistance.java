package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveDistance extends Command {
    
    //Instantializing class variables for the speed, and angle of movement
    private double speed, distance;
    
    //Rotates the robot to the angle 
    public DriveDistance(double speed, double distance){
        this.speed = speed;
        this.distance = distance;
    }

    protected void initialize(){
        Robot.drivetrain.driveDistance(distance, speed);
    }
    protected void execute(){
    }

    //Checks if robot is at desinated angle
    protected boolean isFinished(){
        return Robot.drivetrain.getLeftPID().onTarget() && Robot.drivetrain.getRightPID().onTarget();
    }
}