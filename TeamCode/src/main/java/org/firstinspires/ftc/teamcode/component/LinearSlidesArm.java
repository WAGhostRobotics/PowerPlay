package org.firstinspires.ftc.teamcode.component;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LinearSlidesArm {


    private final double POWER = 1;
    private DcMotor arm;

    public enum TurnValue{
        GROUND(232), // was 180
        BOTTOM(1140), // was 1120
        CONES(1300),
        MID(2005), // was 1972
        TOP(2877); // was 2820

        int ticks;

        TurnValue(int ticks){
            this.ticks = ticks;
        }

        public int getTicks(){
            return ticks;
        }
    }


    //tetrix: 1440 ticks per revolution
    //andymark: 1120 ticks per rev

    //init
    public void init(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "slides");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }



    public void stopArm(){
        arm.setPower(0);
    }

    public void setPower(double power){
        arm.setPower(power);
    }

    public int getTicks(){
        return arm.getCurrentPosition();
    }

    public void moveToPosition(int ticks){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(arm.getCurrentPosition()>ticks){
            multiplier = -1;
        }
        arm.setTargetPosition(ticks);


        //sets power and mode
        arm.setPower(multiplier * POWER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //mover is busy or not
    public boolean isBusy(){
        return arm.isBusy();
    }














    //takes in input location
    public void moveArm(TurnValue location){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(arm.getCurrentPosition()>location.getTicks()){
            multiplier = -1;
        }
        arm.setTargetPosition(location.getTicks());


        //sets power and mode
        arm.setPower(multiplier * 0.92);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //garbage way to determine when to stop mover
//        while(mover.isBusy()){
//
//        }
//        stopMover();
    }


    //move claw up by small increments
    public void moveUp(){
        arm.setTargetPosition(arm.getCurrentPosition() + 40 );
        arm.setPower(0.6);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //move claw down by small increments
    public void moveDown(){
        arm.setTargetPosition(arm.getCurrentPosition() - 40);
        arm.setPower(-0.6);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }









}
