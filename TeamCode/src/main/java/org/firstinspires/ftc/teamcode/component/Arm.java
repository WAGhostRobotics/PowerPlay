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




    public enum TurnValue {
        EXTENDED(0.032),
        ON_THE_FLOOR(0.06),
        SUPER_EXTENDED(0.06),
        START_AUTO(0.835),
        PARTIAL(0.64),
        RETRACTED(0.855),
        LOW(0.53),
        CONE1(0.18),
        CONE2(0.145),
        CONE3(0.105),
        CONE4(0.073),
        CONE5(0.032);

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

    public void moveToPosition(double position){
        targetPos = position;
        arm1.setPosition(position);
        arm2.setPosition(position);

    }

    public boolean isFinished(){

        double encoderPosition = ((-3857.3770491803)*targetPos) + 3343.1475409836;

        return Math.abs(armPosition.getCurrentPosition()-encoderPosition)<=ERROR;
    }


}
