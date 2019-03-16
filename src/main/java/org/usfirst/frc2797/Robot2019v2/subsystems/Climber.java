package org.usfirst.frc2797.Robot2019v2.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {
    //Creating Class Motors

    private final DoubleSolenoid solenoid;

    public Climber(){
        solenoid = new DoubleSolenoid(4,5);

        solenoid.set(Value.kReverse);
    }

    public void initDefaultCommand(){

    }

    public void toggleClimber(){
        solenoid.set((solenoid.get().equals(Value.kForward) ? Value.kReverse:Value.kForward));
    }

    public void extend(){
        solenoid.set(Value.kForward);
    }

    public void retract(){
        solenoid.set(Value.kReverse);
    }
}