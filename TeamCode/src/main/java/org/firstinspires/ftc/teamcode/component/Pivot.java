package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pivot {

    public Servo pivot;

    public final double EXTEND = 0.2;
    public final double PARTIAL = 0.35;
    public final double LOW = 0.3;
    public final double RETRACT = 0.55;
    private final double ERROR =  0.05;

    private double targetPosition = RETRACT;

    public void init (HardwareMap hardwareMap){
        pivot = hardwareMap.get(Servo.class, "pivot");
        retract();
    }

    public void extend(){
        pivot.setPosition(EXTEND);
        targetPosition = EXTEND;
    }

    public void retract(){
        pivot.setPosition(RETRACT);
        targetPosition = RETRACT;
    }

    public void low(){
        pivot.setPosition(LOW);
        targetPosition = LOW;
    }

    public void partial() {
        pivot.setPosition(PARTIAL);
        targetPosition = PARTIAL;
    }

    public boolean isFinished() {
        return Math.abs(pivot.getPosition() - targetPosition) <= ERROR;
    }

    public double getPosition(){
        return pivot.getPosition();
    }
}
