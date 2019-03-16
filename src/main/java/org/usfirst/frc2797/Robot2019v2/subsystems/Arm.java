package org.usfirst.frc2797.Robot2019v2.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem{
    //Instantiating Class Solenoids
    //private final DoubleSolenoid armPistons;
    private final Solenoid armPistons;

    public Arm(){
        //Creating Pistons
        //armPistons = new DoubleSolenoid(2 ,3);
        armPistons = new Solenoid(2);

        armPistons.set(false);
    }

    @Override
    protected void initDefaultCommand(){
    }

    public void extend(){ 
        //Extending Arm
        armPistons.set(true);
     }

    public void retract(){ 
        //Bringing back arm 
        armPistons.set(false);

    }

    public void toggle(){ 
        //Toggling Arms Function
        armPistons.set((!armPistons.get()));

    }
}