package org.firstinspires.ftc.teamcode.component;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class OuttakeSlides {

    private DcMotorEx slides;
    private final double POWER = 1;
    private final double ERROR = 67.5;

    private final double minPower = 0.3;
    private final double maxPower = 1;

    private double stallCurrent = 5.9;


    public enum TurnValue {
        SUPER_RETRACTED(-49),
        RETRACTED(0),
        MID(540),
        ON_THE_WAY_DOWN(700),
        TOP(890), // 880
        AUTO_TOP(890); //880

        int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    public void init(HardwareMap hwMap, boolean teleop) {
       if(teleop){
           slides = hwMap.get(DcMotorEx.class, "outtake");
           slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       }else{
           slides = hwMap.get(DcMotorEx.class, "outtake");
           slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       }

       stallCurrent = hwMap.voltageSensor.iterator().next().getVoltage()/2.2;

        setTargetPosition(0);
    }

    public void setTargetPosition(int targetPos){
        slides.setTargetPosition(targetPos);
    }

    public int getTargetPosition(){
        return slides.getTargetPosition();
    }

    public void update(){
        if(!(slides.getTargetPosition()==OuttakeSlides.TurnValue.SUPER_RETRACTED.getTicks()&&slides.getCurrentPosition()<=0)){
            int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

            if(slides.getCurrentPosition()>slides.getTargetPosition()){
                multiplier = -1;
            }
            //sets power and mode
            slides.setPower(multiplier * POWER);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            slides.setPower(0);
        }

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

    public double getCurrent(){
        return slides.getCurrent(CurrentUnit.AMPS);
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

    public double getAdjustedPower(){
        double power = (((maxPower-minPower)*Math.abs(slides.getTargetPosition()-slides.getCurrentPosition()))/(100)) + minPower;

        return power;
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

    public boolean isStalling(){
        return slides.getCurrent(CurrentUnit.AMPS)>stallCurrent;
    }

    public void stopArm(){
        slides.setPower(0);
    }
}
