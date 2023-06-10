package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo arm1;
    private Servo arm2;
    private DcMotorEx armPosition;
    private final double ERROR = 300;
    private double targetPos = 0;

    //


    public enum TurnValue {
        EXTENDED(0.025),
        ON_THE_FLOOR(0.05),
        SUPER_EXTENDED(0.05),
        START_AUTO(0.825),
        PARTIAL(0.63),
        RETRACTED(0.828),
        LOW(0.52),
        CONE1(0.18),
        CONE2(0.135),
        CONE3(0.105),
        CONE4(0.064),
        CONE5(0.025);

        double position;

        TurnValue(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }



    public void init(HardwareMap hwMap, boolean teleop) {
        arm1 = hwMap.get(Servo.class, "arm1");

        arm2 = hwMap.get(Servo.class, "arm2");
        arm2.setDirection(Servo.Direction.REVERSE);


        if(teleop){
            armPosition = hwMap.get(DcMotorEx.class, "armPosition");
            armPosition.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            armPosition = hwMap.get(DcMotorEx.class, "armPosition");
            armPosition.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    public double getPosition(){
        return armPosition.getCurrentPosition();
    }

    public void setTargetPosition(double position){
        targetPos = position;
    }

    public double getTargetPosition(){
       return targetPos;
    }

    public void update(){
        arm1.setPosition(targetPos);
        arm2.setPosition(targetPos);
    }

    public void moveToPosition(double position){
        targetPos = position;
        arm1.setPosition(position);
        arm2.setPosition(position);

    }

    public boolean isFinished(){

        double encoderPosition = ((-3857.3770491803)*targetPos) + 3343.1475409836;

        return Math.abs(armPosition.getCurrentPosition()-encoderPosition)<=ERROR;
    }

    public double encoderPositionTranslation(){
        return ((-3857.3770491803)*targetPos) + 3343.1475409836;
    }


}
