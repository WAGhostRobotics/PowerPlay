package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public Servo claw;
    public Servo spin;
    private boolean isOpen = false;
    public final double OPEN = 0.08;

    public final double CLOSE = 0.38;

    public final double IN = 0.457;

    public final double OUT = 0.34;

    public double targetPosition = IN;

    private final double ERROR =  0.05;



    public void init(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        spin = hardwareMap.get(Servo.class, "spin");
        claw.setDirection(Servo.Direction.REVERSE);
        open();
        in();
    }

    public void in(){
        spin.setPosition(IN);
        targetPosition = IN;
    }

    public void out(){
        spin.setPosition(OUT);
        targetPosition = OUT;
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
        return spin.getPosition();
    }

    public boolean isFinished() {
        return Math.abs(spin.getPosition() - targetPosition) <= ERROR;
    }

    public boolean isOpen() {
        return isOpen;
    }
}