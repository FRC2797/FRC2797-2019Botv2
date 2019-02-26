package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveElevatorToTop extends Command {
    
    private double speed;
    
    //Moves elevator Up
    public MoveElevatorToTop(double speed){
        this.speed = speed;
    }

    protected void execute(){
        Robot.elevator.moveElevator(speed);
    }

    //Checks if elevator is at top
    protected boolean isFinished(){
        return Robot.elevator.isTop();
    }
}