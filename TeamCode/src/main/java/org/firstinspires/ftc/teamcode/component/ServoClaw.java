package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClaw {

    public Servo claw;
    private boolean isOpen = false;
    public final double OPEN = 0.2;

    public final double CLOSE = 0.55;

    public void init (HardwareMap hardwareMap){

        claw = hardwareMap.get(Servo.class, "claw");

    }

    public void open(){
        claw.setPosition(OPEN);
        isOpen = true;
    }

    public void close(){
        claw.setPosition(CLOSE);
        isOpen = false;
    }

    public double getPosition(){
        return claw.getPosition();
    }

    public boolean isOpen() {
        return isOpen;
    }
}
