package org.usfirst.frc2797.Robot2019v2.commands;

import org.usfirst.frc2797.Robot2019v2.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveElevatorToBottom extends Command{

    private double speed;

    //Moves the elevator down
    public MoveElevatorToBottom(double speed){
        this.speed = -speed;
    }

    public void execute(){
        Robot.elevator.moveElevator(-speed);
    }

    //Checks if the robot is at bottom
    protected boolean isFinished(){
        return Robot.elevator.isBottom(); 
    }
}