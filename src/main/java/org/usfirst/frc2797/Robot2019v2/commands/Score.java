package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;
import org.usfirst.frc2797.Robot2019v2.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Score extends CommandGroup{
    Drivetrain drivetrain = Robot.drivetrain;
    double[] limelightDirections;
    double speed;

    public Score(double speed){
        limelightDirections = drivetrain.getLimelightDirections();
        this.speed = speed;

        addSequential(new RotateToAngle(speed, limelightDirections[2]));
        addSequential(new RotateToAngle(speed, 90));

        addSequential(new DriveDistance(speed, limelightDirections[0]));

        addSequential(new RotateToAngle(speed, 90));

        addSequential(new DriveDistance(speed, limelightDirections[1]));
    }

    protected void execute(){

    }

    protected boolean isFinished(){
        return true;
    }
}