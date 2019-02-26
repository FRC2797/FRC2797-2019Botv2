package org.usfirst.frc2797.Robot2019v2.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Hatch extends Subsystem {

    //Instantiating Class Solenoids
    private final Solenoid hatchPiston1;
    //private final Solenoid hatchPiston2;

    @Override
    protected void initDefaultCommand() {
    }

    public Hatch(){
        //Creating Pistons
        hatchPiston1 = new Solenoid(0);
        //hatchPiston2 = new Solenoid(1);
    }

    public void extend(){
        //Drop Hatch
        hatchPiston1.set(true);
        //hatchPiston2.set(true);
    }

    public void retract(){ 
        //Bring Up Hatch
        hatchPiston1.set(false);
        //hatchPiston2.set(false); 
    }

    public void toggle(){ 
        //Toggling hatch functions
        hatchPiston1.set(!hatchPiston1.get()); 
        //hatchPiston2.set(!hatchPiston2.get());
    }
}