package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo arm1;
    private Servo arm2;
    private final double ERROR = 0.01;
    private double targetPos = 0;




    public enum TurnValue {



        EXTENDED(1),
        PARTIAL(0.1),
        RETRACTED(0),
        LOW(0.5),
        CONE1(0.96),
        CONE2(0.97),
        CONE3(0.98),
        CONE4(0.99),
        CONE5(1);

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
//        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2 = hwMap.get(Servo.class, "arm2");

    }

    public double getPosition(){
        return arm1.getPosition();
    }

    public void moveToPosition(double position){
        targetPos = position;
        arm1.setPosition(position);
        arm2.setPosition(position);

    }

    public boolean isFinished(){
        return Math.abs(arm1.getPosition()-targetPos)<=ERROR;
    }


}
