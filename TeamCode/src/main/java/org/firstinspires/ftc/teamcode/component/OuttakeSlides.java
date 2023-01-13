package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSlides {

    private DcMotor slides;
    private final double POWER = 1;
    private final double ERROR = 15;


    public enum TurnValue {
        RETRACTED(0),
        MID(1931),
        TOP(2700);

        int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    public void init(HardwareMap hwMap) {
       slides = hwMap.get(DcMotor.class, "outtake");
       slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveToPosition(int ticks){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(slides.getCurrentPosition()>ticks){
            multiplier = -1;
        }
        slides.setTargetPosition(ticks);


        //sets power and mode
        slides.setPower(multiplier * POWER);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // move claw up by small increments
    public void moveUp(){
        slides.setTargetPosition(slides.getCurrentPosition() + 40 );
        slides.setPower(0.6);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // move claw down by small increments
    public void moveDown(){
        slides.setTargetPosition(slides.getCurrentPosition() - 40);
        slides.setPower(-0.6);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getTicks(){
        return slides.getCurrentPosition();
    }

    //mover is busy or not
    public boolean isBusy(){
        return slides.isBusy();
    }

    public boolean isFinished(){
        return Math.abs(slides.getCurrentPosition()-slides.getTargetPosition())<=ERROR;
    }

    public void stopArm(){
        slides.setPower(0);
    }
}
