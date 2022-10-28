package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServo roller1;
    private CRServo roller2;
    private final double POWER = 1;

    public void init (HardwareMap hardwareMap) {
        roller1 = hardwareMap.get(CRServo.class, "roller1");
        roller2 = hardwareMap.get(CRServo.class, "roller2");
        roller2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void in() {
        roller1.setPower(-POWER);
        roller2.setPower(-POWER);
    }

    public void out() {
        roller1.setPower(POWER);
        roller2.setPower(POWER);
    }

    public void stop() {
        roller1.setPower(0);
        roller2.setPower(0);
    }
}
