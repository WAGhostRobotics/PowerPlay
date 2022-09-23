package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    private DcMotor clawMotor;

    private final int OPEN = 241;
    private final int CLOSE = -6;

    private final double POWER = 0.4;
    private final double SLOW = 0.1;

    private final int INCREMENT = 10;

    public void init(HardwareMap hardwareMap){

        clawMotor = hardwareMap.get(DcMotor.class, "claw");

        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        clawMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        reset();
    }

    private void reset(){
        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void open(){
        clawMotor.setTargetPosition(OPEN);

        clawMotor.setPower(POWER);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void close(){
        clawMotor.setTargetPosition(CLOSE);
        clawMotor.setPower(-POWER);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void openSlowly(){
        clawMotor.setTargetPosition(clawMotor.getCurrentPosition()+INCREMENT);

        clawMotor.setPower(SLOW);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void closeSlowly(){
        clawMotor.setTargetPosition(clawMotor.getCurrentPosition()-INCREMENT);
        clawMotor.setPower(-SLOW);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopClaw(){
        clawMotor.setPower(0);
    }

    public int getTicks(){
        return clawMotor.getCurrentPosition();
    }

    public boolean isBusy(){
        return clawMotor.isBusy();
    }



}
