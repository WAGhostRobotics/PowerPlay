package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public Servo claw;
    public Servo spin;

    public static final double OPEN = 0.08;

    public static final double CLOSE = 0.38;

    public static final double IN = 0.454;

    public static final double OUT = 0.336;

    private double spinTargetPosition = IN;

    private double clawTargetPosition = CLOSE;

    private final double ERROR =  0.01;



    public void init(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        spin = hardwareMap.get(Servo.class, "spin");
        claw.setDirection(Servo.Direction.REVERSE);
        close();
        in();
    }

    public double getSpinTargetPosition(){
        return spinTargetPosition;
    }

    public double getClawTargetPosition(){
        return clawTargetPosition;
    }

    public void in(){
        spin.setPosition(IN);
        spinTargetPosition = IN;
    }

    public void out(){
        spin.setPosition(OUT);
        spinTargetPosition = OUT;
    }

    public void setClawPosition(double position){
        claw.setPosition(position);
        clawTargetPosition = position;
    }

    public  void setSpinPosition(double position){
        spin.setPosition(position);
        spinTargetPosition = position;
    }


    public void open(){
        claw.setPosition(OPEN);
        clawTargetPosition = OPEN;
    }

    public void close(){
        claw.setPosition(CLOSE);
        clawTargetPosition = CLOSE;
    }


    public boolean isIn(){
        return spinTargetPosition == IN;
    }


    public boolean spinIsFinished() {
        return Math.abs(spin.getPosition() - spinTargetPosition) <= ERROR;
    }

    public boolean clawIsFinished() {
        return Math.abs(claw.getPosition() - clawTargetPosition) <= ERROR;

    }

    public boolean isOpen() {
        return clawTargetPosition==OPEN;
    }
}