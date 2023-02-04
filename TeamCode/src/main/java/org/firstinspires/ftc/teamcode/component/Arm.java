package org.firstinspires.ftc.teamcode.component;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Arm {
    private CRServo arm1;
    private CRServo arm2;
    private DcMotor armPosition;
    private final double POWER = 1;
    private final double ERROR = 100;
    private int targetPos = 0;


    private double maxPower = 0.8;
    private double minPower = 0.05;

    private int MIN = 0;
    private int MAX = 2000;


    public enum TurnValue {



        EXTENDED(3050),
        PARTIAL(250),
        RETRACTED(-150),
        LOW(1610);

        int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }



    public void init(HardwareMap hwMap) {
        arm1 = hwMap.get(CRServo.class, "arm1");
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2 = hwMap.get(CRServo.class, "arm2");
        armPosition = hwMap.get(DcMotor.class, "armPosition");
        armPosition.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void moveToPosition(int ticks, double power){
        armPosition.setTargetPosition(ticks); // useless line of code
        targetPos = ticks;


        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(armPosition.getCurrentPosition()>ticks){
            multiplier = -1;
        }


        //sets power and mode
        setPower(multiplier * power);

    }

    public void updateTargetPos(int targetPos){
        this.targetPos =  targetPos;
    }

    public double getAdjustedPower(int targetPos){
        double power = (((maxPower-minPower)*Math.abs(targetPos-armPosition.getCurrentPosition()))/(MAX-MIN)) + minPower;




        return power;
    }





    public boolean isBusy(){
        return armPosition.isBusy();
    }

    public int getTicks(){
        return armPosition.getCurrentPosition();
    }



    public boolean isFinished(){
        return Math.abs(armPosition.getCurrentPosition()-targetPos)<=ERROR;
    }

    public void stopArm(){
        setPower(0);
    }

    public void setPower(double power){
        arm1.setPower(power);
        arm2.setPower(power);
    }
}
