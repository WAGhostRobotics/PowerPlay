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
        EXTENDED(0.04),
        ON_THE_FLOOR(0.07),
        SUPER_EXTENDED(0.07),
        START_AUTO(0.83),
        PARTIAL(0.65),
        RETRACTED(0.84),
        LOW(0.54),
        CONE1(0.18),
        CONE2(0.16),
        CONE3(0.115),
        CONE4(0.07),
        CONE5(0.04);

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

        double encoderPosition = ((-3847.2222222222)*targetPos) + 3400.8888888889;

        return Math.abs(armPosition.getCurrentPosition()-encoderPosition)<=ERROR;
    }


}
