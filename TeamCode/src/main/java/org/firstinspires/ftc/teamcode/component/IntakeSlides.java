package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSlides {
    private DcMotor slides;
    private final double POWER = 1;
    private final double ERROR = 15;





    public enum TurnValue {
        RETRACTED(0),
        PLACE_CONE(100),
        PARTIAL(1931),
        EXTENDED(2700);

        int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    private final double MAX = TurnValue.EXTENDED.getTicks();
    private final double MIN = TurnValue.RETRACTED.getTicks();

    private final double minPower = 0.3;
    private final double maxPower = 1;

    public double getAdjustedPower(){
        double power = (((maxPower-minPower)*(slides.getCurrentPosition()-MIN))/(MAX-MIN)) + minPower;

        return power;
    }

    public void init(HardwareMap hwMap) {
        slides = hwMap.get(DcMotor.class, "intake");
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

    public void moveToPosition(int ticks, double power){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(slides.getCurrentPosition()>ticks){
            multiplier = -1;
        }
        slides.setTargetPosition(ticks);


        //sets power and mode
        slides.setPower(multiplier * power);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // extends slides by small increments
    public void moveUp() {
        slides.setTargetPosition(slides.getCurrentPosition() + 40 );
        slides.setPower(0.6);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // retracts claws by small increments
    public void moveDown() {
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
