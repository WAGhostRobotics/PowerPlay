package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Latch {

    public Servo latch;

    public static final double OPEN = 0.5;

    public static final double CLOSE = 0.06;

    private double latchTargetPosition = OPEN;

    private final double ERROR =  0.01;



    public void init(HardwareMap hardwareMap){
        latch = hardwareMap.get(Servo.class, "latch");
    }

    public void setLatchPosition(double position){
        latch.setPosition(position);
        latchTargetPosition = position;
    }

    public double getLatchPosition(){
        return latchTargetPosition;
    }
}