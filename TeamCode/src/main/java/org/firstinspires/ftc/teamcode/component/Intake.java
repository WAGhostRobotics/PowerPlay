package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor roller1;
    private DcMotor roller2;
    private final double POWER = 1;

    public void init (HardwareMap hardwareMap) {
        roller1 = hardwareMap.get(DcMotor.class, "roller1");
        roller2 = hardwareMap.get(DcMotor.class, "roller2");
        roller1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void in() {
        roller1.setPower(POWER);
        roller2.setPower(-POWER);
    }

    public void out() {
        roller1.setPower(-POWER);
        roller2.setPower(POWER);
    }

    public void stop() {
        roller1.setPower(0);
        roller2.setPower(0);
    }
}
