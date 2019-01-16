package org.usfirst.frc2797.Robot2019v2.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

    private Spark leftMotor;
    private Spark rightMotor;

    private SpeedControllerGroup climberMotors;

    public Climber(){
        leftMotor = new Spark(4);
        addChild("leftMotor", leftMotor);
        leftMotor.setInverted(false);

        rightMotor = new Spark(5);
        addChild("rightMotor", rightMotor);
        rightMotor.setInverted(true);

        climberMotors = new SpeedControllerGroup(leftMotor, rightMotor);
        addChild("climberMotors", climberMotors);

    }

    public void initDefaultCommand(){

    }

    @Override
    public void periodic(){

    }

    public void driveClimber(double speed){
        climberMotors.set(speed);
    }
}