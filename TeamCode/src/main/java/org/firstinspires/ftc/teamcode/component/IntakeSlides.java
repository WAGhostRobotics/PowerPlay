package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSlides {
    private DcMotorEx slides;
    private final double POWER = 0.55;
    private final double ERROR = 19;





    public enum TurnValue {
        RETRACTED(9),
        PLACE_CONE(9),
        AUTO_STACK(95),
        PARTIAL(30),
        ALMOST_DONE(150),
        EXTENDED(425),
        AUTO_EXTENDED(282),//280
        AUTO_EXTENDED_LEFT(275);


        int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    private final double MAX = 50;
    private final double MIN = 0;

    private final double minPower = 0.5;
    private final double maxPower = 1;

    public double getAdjustedPower(){
        double power = (((maxPower-minPower)*(Math.abs(slides.getTargetPosition()-slides.getCurrentPosition())-MIN))/(MAX-MIN)) + minPower;


        if(power>=1){
            power = 1;
        }else if(power<=0){
            power = 0;
        }


        return power;
    }

    public double getCurrent(){
        return slides.getCurrent(CurrentUnit.AMPS);
    }



    public void init(HardwareMap hwMap, boolean teleop) {
        if(teleop){
            slides = hwMap.get(DcMotorEx.class, "intake");
            slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            slides = hwMap.get(DcMotorEx.class, "intake");
            slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setTargetPosition(0);
    }

    public void setTargetPosition(int targetPos){
        slides.setTargetPosition(targetPos);
    }

    public int getTargetPosition(){
        return slides.getTargetPosition();
    }



    public void update(){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(slides.getCurrentPosition()>slides.getTargetPosition()){
            multiplier = -1;
        }
        //sets power and mode

//        double power = getAdjustedPower();
        slides.setPower(multiplier * POWER);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
