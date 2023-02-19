package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo arm1;
    private Servo arm2;
    private DcMotor armPosition;
    private final double ERROR = 300;
    private double targetPos = 0;




    public enum TurnValue {
        EXTENDED(0.02),
        SUPER_EXTENDED(0.04),
        PARTIAL(0.66),
        RETRACTED(0.82),
        LOW(0.55),
        CONE1(0.17),
        CONE2(0.13),
        CONE3(0.10),
        CONE4(0.06),
        CONE5(0.02);

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
            armPosition = hwMap.get(DcMotor.class, "armPosition");
        }else{
            armPosition = hwMap.get(DcMotor.class, "armPosition");
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

        double encoderPosition = ((-3851.5625)*targetPos) + 3224.03125;

        return Math.abs(armPosition.getCurrentPosition()-encoderPosition)<=ERROR;
    }


}
