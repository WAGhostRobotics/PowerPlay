package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClaw {

    public Servo claw;

    public final double OPEN = 0.2;

    public final double CLOSE = 0.6;

    public void init (HardwareMap hardwareMap){

        claw = hardwareMap.get(Servo.class, "claw");

    }

    public void open(){
        claw.setPosition(OPEN);
    }

    public void close(){
        claw.setPosition(CLOSE);
    }

    public double getPosition(){
        return claw.getPosition();
    }
}
