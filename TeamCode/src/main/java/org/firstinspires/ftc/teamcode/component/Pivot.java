package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pivot {

    public Servo pivot1;
    public Servo pivot2;

    public final double EXTEND = 0.2;
    public final double PARTIAL = 0.35;
    public final double LOW = 0.3;
    public final double RETRACT = 0.55;
    private final double ERROR =  0.05;

    private double targetPosition = RETRACT;

    public void init (HardwareMap hardwareMap){
        pivot1 = hardwareMap.get(Servo.class, "pivot1");
        pivot2 = hardwareMap.get(Servo.class, "pivot2");
        retract();
    }

    public void extend(){
        setPosition(EXTEND);
        targetPosition = EXTEND;
    }

    public void retract(){
        setPosition(RETRACT);
        targetPosition = RETRACT;
    }

    public void low(){
        setPosition(LOW);
        targetPosition = LOW;
    }

    public void partial() {
        setPosition(PARTIAL);

        targetPosition = PARTIAL;
    }

    public void setPosition(double position){
        pivot1.setPosition(position);
        pivot2.setPosition(position);
    }

    public boolean isFinished() {
        return Math.abs(pivot1.getPosition() - targetPosition) <= ERROR&&Math.abs(pivot2.getPosition() - targetPosition) <= ERROR;
    }

}
