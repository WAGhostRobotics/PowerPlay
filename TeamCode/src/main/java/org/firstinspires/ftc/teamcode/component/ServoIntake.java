package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoIntake {

    private CRServo servoIntake;
    private final double POWER = 1;

    public void init(HardwareMap hardwareMap) {
        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");
        servoIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void in() {
        servoIntake.setPower(POWER);
    }

    public void out() {
        servoIntake.setPower(-POWER);
    }

    public void stop() {
        servoIntake.setPower(0);
    }

}
