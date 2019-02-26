package org.usfirst.frc2797.Robot2019v2.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc2797.Robot2019v2.Robot;
import org.usfirst.frc2797.Robot2019v2.commands.TeleopDrive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This subsystem is for the robot's drivetrain. It contains all the methods used
 * for controlling the movement of the robot.
 */
public class Drivetrain extends Subsystem implements PIDOutput {
    
    //Right side motor controllers
    private final VictorSPX frontRight;
    private final VictorSPX rearRight;

    //Left side motor controllers
    private final VictorSPX frontLeft;
    private final VictorSPX rearLeft;

    //Drivetrain encoders
    private final Encoder leftEnc;
    private final Encoder rightEnc;

    //Gyroscope
    private final AHRS navX;

    //PIDControllers
    private final PIDController leftPID;
    private final PIDController rightPID;
    private final PIDController gyroPID;
    
    //The variable to output to for the gyroscope PIDController
    double rotateToAngleRate;

    //Constants used to calculate the setpoints for the drivetrain PIDControllers
    private final double wheelDiameter = 0.5;
    private final double wheelCircumference = wheelDiameter*Math.PI;

    //Dummy sparks used as outputs for the left and right drivetrain PIDControllers
    private final static Spark leftDummy = new Spark(10);
    private final static Spark rightDummy = new Spark(11);

    //Data received from the limelight
    private NetworkTable table = Robot.table;
    private  double x, y, area;


    public Drivetrain() {

        /**Initialize the motor controllers for the drivetrain */
        
        //Right side motor controllers
        frontRight = new VictorSPX(2);
        frontRight.setInverted(true);
        
        rearRight = new VictorSPX(3);
        rearRight.setInverted(true);

        rearRight.follow(frontRight);
        
        //Left side motor controllers
        frontLeft = new VictorSPX(0);
        frontLeft.setInverted(false);
        
        rearLeft = new VictorSPX(1);
        rearLeft.setInverted(false);

        rearLeft.follow(frontLeft);
        
        /**Initialize the encoders and gyroscope */
        //The encoder on the left side of the drivetrain
        leftEnc = new Encoder(0, 1, false, EncodingType.k2X);
        addChild("Drivetrain Left Encoder", leftEnc);
        leftEnc.setDistancePerPulse(1.0);
        leftEnc.setPIDSourceType(PIDSourceType.kDisplacement);
        SmartDashboard.putData("Left Encoder", leftEnc);

        //The encoder on the right side of the drivetrain
        rightEnc = new Encoder(2, 3, false, EncodingType.k2X);
        addChild("Drivetrain Right Encoder", rightEnc);
        rightEnc.setDistancePerPulse(1.0);
        rightEnc.setPIDSourceType(PIDSourceType.kDisplacement);
        SmartDashboard.putData("Right Encoder", rightEnc);

        //The gyroscope
        navX = new AHRS(SPI.Port.kMXP);
        addChild("navX", navX);

        /**Create the PIDControllers for closed-loop control */
        //Controller for the left motors
        leftPID = new PIDController(0.0075, 0.01, 0.0, 0.0, leftEnc, leftDummy);
        addChild("Drivetrain Left PID", leftPID);
        leftPID.setAbsoluteTolerance(10.0);
        leftPID.setInputRange(Double.MIN_VALUE, Double.MAX_VALUE);
        leftPID.setOutputRange(-1.0, 1.0);
        leftPID.setContinuous(false);

        //Controller for the right motors
        rightPID = new PIDController(0.0075, 0.01, 0.0, 0.0, rightEnc, rightDummy);
        addChild("Drivetrain Right PID", rightPID);
        rightPID.setAbsoluteTolerance(10.0);
        rightPID.setInputRange(Double.MIN_VALUE, Double.MAX_VALUE);
        rightPID.setOutputRange(-1.0, 1.0);
        rightPID.setContinuous(false);
 
        //Controller for gyroscope (robot rotation)
        gyroPID = new PIDController(0.065, 0.1, 0.15, 0.5, navX, this);
        addChild("Drivetrain Gyro PID", gyroPID);
        gyroPID.setAbsoluteTolerance(0.5);
        gyroPID.setInputRange(-360.0, 360.0);
        gyroPID.setOutputRange(-1.0, 1.0);
        gyroPID.setContinuous(false);
    }

    @Override
    public void initDefaultCommand() {
        //Set teleop as the default command
        setDefaultCommand(new TeleopDrive());
    }

    @Override
    public void periodic() {
        /**Limelight Code */
        //Put data in a network table
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //Read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        //Post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        /**NavX Dashboard Code */
        SmartDashboard.putNumber("navX Yaw", navX.getYaw());
    }


    //Methods for controlling the drivetrain

    /**
     * Drive the robot using the input speed values.
     * @param leftSpeed The speed of the left wheels.
     * @param rightSpeed The speed of the right wheels.
     */
    public void drive(double leftSpeed, double rightSpeed){
        frontRight.set(ControlMode.PercentOutput, rightSpeed);
        frontLeft.set(ControlMode.PercentOutput, leftSpeed);
    }

    /**
     * Reset the yaw on the navx.
     */
    public void resetYaw(){
        navX.zeroYaw();
    }

    /**
     * Reset the drivetrain encoders.
     */
    public void resetEncoders(){
        leftEnc.reset();
        rightEnc.reset();
    }

    /**
     * Get the gyroPID for use in commands.
     */
    public PIDController getGyroPID(){
        return gyroPID;
    }

    /**
     * Disables all PIDControllers in the drivetrain
     */
    public void disablePIDControllers(){
        leftPID.disable();
        rightPID.disable();
        gyroPID.disable();
    }
    
    /**
     * Turn the robot by a specific degree measure. This is relative to the current rotation.
     * @param angle The angle of rotation (in degrees).
     * @param speed The maximum speed at which to rotate (0.0-1.0).
     */
    public void rotateToAngle(double angle, double speed){
        //Set the output range of the gyroPID to the value of 'speed'.
        gyroPID.setOutputRange(-speed, speed);

        //Disable all PIDControllers
        disablePIDControllers();

        //Zero the yaw (y-axis rotation) of the navx.
        resetYaw();

        //Change the setpoint of the gyroPID to the value of 'angle'.
        gyroPID.setSetpoint(angle);

        //Set this variable to zero. For reasons.
        rotateToAngleRate = 0.0;

        //Enable the gyroPID.
        gyroPID.enable();

        //While the gyroPID is enabled...
        while(gyroPID.isEnabled()){
            //...Rotate the robot.
            drive(rotateToAngleRate, -rotateToAngleRate);

            //When the setpoint is reached, disable the gyroPID.
            if(gyroPID.onTarget())
                gyroPID.disable();
        }

        //Reset yaw again. For funsies.
        resetYaw();
    }

    /**
    * Automatically drive a specific distance. This is relative to the current position.
    * @param distance The distance to be driven (in feet).
    * @param speed The maximum speed at which to drive (0.0-1.0).
    */
    public void driveDistance(double distance, double speed){
        //Set the output ranges of leftPID and rightPID to the value of 'speed'.
        leftPID.setOutputRange(-speed, speed);
        rightPID.setOutputRange(-speed, speed);

        //Disable all PIDControllers.
        disablePIDControllers();

        //Zero the encoders.
        resetEncoders();

        //Change the leftPID and rightPID setpoints using an equation that takes the input distance and turns it into an encoder count.
        leftPID.setSetpoint((int)(distance*(250/wheelCircumference)));
        rightPID.setSetpoint((int)(distance*(250/wheelCircumference)));

        //Enable leftPID and rightPID.
        leftPID.enable();
        rightPID.enable();

        //Disable each PIDController once it reaches its setpoint.
        while(leftPID.isEnabled() || rightPID.isEnabled()){
            drive(leftDummy.get(), rightDummy.get());
            if(leftPID.onTarget())
                leftPID.disable();
            if(rightPID.onTarget())
                rightPID.disable();
        }
    }

    /**
     * Make the robot rotate to center on the currently highlighted target.
     * @param tolerance The tolerance in how precise the robot should be when rotating
     * @param speed The speed at which to rotate the robot
     */
    public void rotateToTarget(double tolerance, double speed){
        // while(x < 0 - tolerance || x > 0 + tolerance){
        //     if(x < 0 - tolerance){
        //         drive(speed, -speed);
        //     }
        //     if(x > 0 + tolerance){
        //         drive(-speed, speed);   
        //     }
        // }
            rotateToAngle(x, speed);
        

    }
    
    /**
     * The method used for the gyroscope 
     * PIDController to write to the drivetrain
     */
    public void pidWrite(double output){
        rotateToAngleRate = output;
    }

    /**
     * 
     * @param speed
     */
    public void mathCalc(double speed){
         //|53.34 is wrong and needs to be fixed
         //|75.565 tape from ground
         //|12 mounting
         double tapeHeight = 75.565;
         double robotHeight = 53.34;
         double angleOfLimelight = 80;
         double a2 = 90 - angleOfLimelight;
         //rough estimate
         double xr =  13;
         double yr = 0;
         double zr = 0;

         

         //TODO these numbers may need to be changed cause Jack is bad
         double distance1 = (tapeHeight - robotHeight) / Math.tan(angleOfLimelight-a2);

         double distance3 = (Math.tan(a2) * distance1); 

         double distance2 = (Math.cos(a2) * distance3);
         
         //drive certain distance 


    }
}