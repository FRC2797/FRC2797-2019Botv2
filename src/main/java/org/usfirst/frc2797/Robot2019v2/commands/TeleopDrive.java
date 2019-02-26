package org.usfirst.frc2797.Robot2019v2.commands;
 
import org.usfirst.frc2797.Robot2019v2.OI;
import org.usfirst.frc2797.Robot2019v2.Robot;
import org.usfirst.frc2797.Robot2019v2.subsystems.Arm;
import org.usfirst.frc2797.Robot2019v2.subsystems.Climber;
import org.usfirst.frc2797.Robot2019v2.subsystems.Drivetrain;
import org.usfirst.frc2797.Robot2019v2.subsystems.Elevator;
import org.usfirst.frc2797.Robot2019v2.subsystems.Hatch;
import org.usfirst.frc2797.Robot2019v2.subsystems.LandingGear;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The Command used during Teleop to control the robot.
 */
public class TeleopDrive extends Command {

    
    private Drivetrain drivetrain = Robot.drivetrain;
    private Elevator elevator = Robot.elevator;
    private Hatch hatch = Robot.hatch;
    private Arm  arm = Robot.arm;
    private Climber climber = Robot.climber;
    private LandingGear landingGear = Robot.landingGear;
    private XboxController xbx = OI.xbx;
    private Hand left = Hand.kLeft, right = Hand.kRight;
    
    public TeleopDrive() {

        requires(Robot.drivetrain);

    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        //Drive the robot
        drivetrain.drive(-xbx.getY(left), -xbx.getY(right));
        
        
        //Move Elevator Up
        if(xbx.getTriggerAxis(right) > 0.1)
            elevator.moveElevator(xbx.getTriggerAxis(right));
        //Move Elevator Down
        if(xbx.getTriggerAxis(left) > 0.1)
            elevator.moveElevator(-xbx.getTriggerAxis(left));
        //Stop Elevator
        if(xbx.getTriggerAxis(right) <= 0.1 && xbx.getTriggerAxis(left) <= 0.1)
            elevator.moveElevator(0);
        
        
        //arm toggle
        if(xbx.getAButtonPressed()){
            arm.toggle();
        }


        //extending hatch
        if(xbx.getBumperPressed(right)){
            hatch.extend();
        }
        //retrating hatch 
        if(xbx.getBumperPressed(left)){
            hatch.retract();
        }


        //Climb
        if(xbx.getXButtonPressed()){
            climber.toggleClimber();
        }

        //Actuate Landing Actuators
        if(xbx.getBButtonPressed()){
            landingGear.toggle();
        }

        //End all  tasks
        if (xbx.getStartButton()){ 
            Scheduler.getInstance().removeAll();

        }

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Scheduler.getInstance().removeAll();  
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Scheduler.getInstance().removeAll();
    }
}
