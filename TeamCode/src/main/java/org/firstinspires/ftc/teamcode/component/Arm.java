package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Arm {
    private CRServo arm1;
    private CRServo arm2;
    private Encoder armPosition;
    private final double POWER = 1;
    private final double ERROR = 15;
    private int targetPos = 0;


    public enum TurnValue {



        EXTENDED(2700),
        PARTIAL(1931),
        RETRACTED(0),
        LOW(0);

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
        arm2 = hwMap.get(CRServo.class, "arm2");
        armPosition = hwMap.get(Encoder.class, "armPosition");
    }

    public void moveToPosition(int ticks){
        targetPos = ticks;

        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(armPosition.getCurrentPosition()>ticks){
            multiplier = -1;
        }



        //sets power and mode
        setPower(multiplier * POWER);

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
