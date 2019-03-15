package org.usfirst.frc2797.Robot2019v2;


public class MathCalc {
    public void rocket (){

        double robotHeight = 59.055;
        double tapeHeight = 53.34;
        double angleOfLimelight = 80.0;
        double a2 = angleOfLimelight - 90.0;
        double robotSize = 81.28;

        double distance1 = (tapeHeight - robotHeight) / Math.tan(angleOfLimelight + a2);
        double distance3 = (Math.cos(a2) * distance1);
        double distance2 = ((Math.tan(a2) * distance3) - robotSize);
    }
    public void score(){
        // my pseudo-code cause I dont know how to do this stuff
        //Robot.rotate(angleOfLimelight + a2);
        //Robot.drive(distance2);
        //Robot.rotate(-90);
        //Robot.drive(distance3);
    }
}