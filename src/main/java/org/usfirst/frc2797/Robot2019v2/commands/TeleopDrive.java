package org.usfirst.frc2797.Robot2019v2.commands;

//region Imports
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
//endregion Imports

/**
 * The Command used during Teleop to control the robot.
 */
public class TeleopDrive extends Command {

    //region Robot Components
    private Drivetrain drivetrain = Robot.drivetrain;
    private Elevator elevator = Robot.elevator;
    private Hatch hatch = Robot.hatch;
    private Arm  arm = Robot.arm;
    private Climber climber = Robot.climber;
    private LandingGear landingGear = Robot.landingGear;
    private XboxController xbx = OI.xbx;
    private Hand left = Hand.kLeft, right = Hand.kRight;
    //endregion Robot Components

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

        //region Drive
        //Drive the robot
        //The If statement if for controller deadzoning
        if(Math.abs(xbx.getY(left)) > 0.1 || Math.abs(xbx.getY(right)) > 0.1){
            drivetrain.drive(-xbx.getY(left), -xbx.getY(right));
        }
        else
            drivetrain.drive(0,0);
        //endregion Drive
        
        //region Elevator

        //Move Elevator Down
        if(xbx.getTriggerAxis(left) > 0.1){
            elevator.moveElevator(xbx.getTriggerAxis(left)*.7);
            System.out.println("right move");
        }
        //Move Elevator Up
        if(xbx.getTriggerAxis(right) > 0.1){
            elevator.moveElevator(-xbx.getTriggerAxis(right));
            System.out.println("left move");
        }
        //Stop Elevator
        if(xbx.getTriggerAxis(right) <= 0.1 && xbx.getTriggerAxis(left) <= 0.1){
            elevator.moveElevator(-xbx.getTriggerAxis(right));
        }
        
        //endregion Elevator
        
        //region Arm
        //arm toggle
        if(xbx.getAButtonPressed()){
            arm.toggle();
        }
        //endregion Arm

        //region Hatch
        //extending hatch
        if(xbx.getBumperPressed(left)){
            hatch.extend();
        }
        //retrating hatch 
        if(xbx.getBumperPressed(right)){
            hatch.retract();
        }
        //endregion Hatch

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
        
        //very SCARY code, PID may need tuning i dont know, robot tends to overshoot target 
        if(xbx.getYButtonPressed()){
            drivetrain.rotateToTarget(0.4, 0.9);
        }

        //super SCARY code, needs editing probably 
        if(xbx.getPOV()==180){
            drivetrain.driveToTarget(.1);
            //new Score(0.2);    
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
