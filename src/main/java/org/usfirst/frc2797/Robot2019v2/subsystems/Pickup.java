package org.usfirst.frc2797.Robot2019v2.subsystems;
/*
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

//This dont exist no more
public class Pickup extends Subsystem{
    //Creating Class motors
    private final VictorSPX pickup1;
    private final VictorSPX pickup2;

    @Override
    protected void initDefaultCommand() {
    }

    public Pickup(){
        //Initializing Motors
        //pickup1 = new VictorSPX(6);
        //pickup2 = new VictorSPX(7);

        //Master-Slave
        pickup2.follow(pickup1);
    }

    public void movePickup(double speed){
        //Picks up
        pickup1.set(ControlMode.PercentOutput, speed);
    }

}*/