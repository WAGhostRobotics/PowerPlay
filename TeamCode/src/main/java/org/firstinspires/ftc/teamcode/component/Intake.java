package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private CRServo roller1;
    private CRServo roller2;
    private DistanceSensor coneSensor;
    private TouchSensor wallSensor;


    private final double POWER = 1;
    private final double DISTANCE = 3.35;

    public void init (HardwareMap hardwareMap) {
        roller1 = hardwareMap.get(CRServo.class, "roller1");
        roller2 = hardwareMap.get(CRServo.class, "roller2");
        coneSensor = hardwareMap.get(DistanceSensor.class, "sensor");
        wallSensor = hardwareMap.get(TouchSensor.class, "touch");

        roller1.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public boolean hasFreight() {
        return coneSensor.getDistance(DistanceUnit.CM) < DISTANCE;
    }

    public double getDistance() {
        return coneSensor.getDistance(DistanceUnit.CM);
    }

    public boolean isTouchingWall(){
        return wallSensor.isPressed();
    }
}
