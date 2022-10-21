package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    private DcMotor arm;
    private final double POWER = 1;

    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void up() {
        arm.setPower(POWER);
    }

    public void down() {
        arm.setPower(-POWER);
    }

    public void stop() {
        arm.setPower(0);
    }

}
