package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ServoIntake {

    private CRServo servoIntake;
    private DistanceSensor sensor;
    private final double POWER = 1;

    public void init(HardwareMap hardwareMap) {
        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");
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
