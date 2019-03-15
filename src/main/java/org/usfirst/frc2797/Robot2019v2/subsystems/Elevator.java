package org.usfirst.frc2797.Robot2019v2.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

public class Elevator {
    //Creating Class motor
   private final VictorSPX elevator1;
   private final VictorSPX elevator2;

   //Creating Class Limit Switches
   //private final DigitalInput topSwitch;
   //private final DigitalInput bottomSwitch;

   //Creating Class Encoder
   private final Encoder elevEnc;
   
   private final Spark dummy;

   //Class PID for elevator
   private final PIDController elevatorPID;


   public Elevator(){
       //Initializing the motors 
        elevator1 = new VictorSPX(4);
        elevator2 = new VictorSPX(5);
        //Master-Slave functionality 
        elevator2.follow(elevator1);
        elevator2.setInverted(true);
       
        //Initializing the Limit Switch's
        //bottomSwitch = new DigitalInput(4);
        //topSwitch = new DigitalInput(5);   
        
        //Initializing the Encoder
        elevEnc = new Encoder(6, 7, false, EncodingType.k2X);

        dummy = new Spark (12);

        //Fun PIDS
        elevatorPID = new PIDController(0.0075, 0.01, 0.0, 0.0, elevEnc, dummy);
        elevatorPID.setAbsoluteTolerance(10.0);
        elevatorPID.setOutputRange(-1.0, 1.0);
        elevatorPID.setContinuous(false);
   }    

   public void moveElevator (double speed){
       //Moving the elevator UP or DOWN 
      elevator1.set(ControlMode.PercentOutput, speed);
   }

   public boolean isTop(){
       //Checks if the elevator is at the top
       return false; //topSwitch.get();
   }

   public boolean isBottom(){
       //Checks if the elevator is at the bottom
       return false; //bottomSwitch.get();
   }

}

