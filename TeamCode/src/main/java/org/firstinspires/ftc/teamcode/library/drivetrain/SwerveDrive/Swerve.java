package org.firstinspires.ftc.teamcode.library.drivetrain.SwerveDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;

public class Swerve implements Drivetrain {

    private ModuleV2 frontLeft, frontRight, backRight, backLeft;

    private final double TRACK_WIDTH = 10;
    private final double WHEEL_BASE = 10;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public Swerve(HardwareMap hwMap, boolean reset){
        frontLeft = new ModuleV2(hwMap, "lf", "lfPivot", "lfEnc", reset);
        frontRight = new ModuleV2(hwMap, "rf", "rfPivot", "rfEnc", reset);
        backRight = new ModuleV2(hwMap, "rr", "rrPivot", "rrEnc", reset);
        backLeft = new ModuleV2(hwMap, "lr", "lrPivot", "lrEnc", reset);
    }


    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower) {
        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(frontLeftPower, Math.max(frontRightPower, Math.max(backLeftPower, backRightPower)));

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        frontLeft.setPower(movementPower * frontLeftPower);
        frontRight.setPower(movementPower * frontRightPower);
        backLeft.setPower(movementPower * backLeftPower);
        backRight.setPower(movementPower * backRightPower);

    }

    public void driveCommon(double magnitude, double theta, double driveTurn){
        double x = magnitude * Math.cos(Math.toRadians(theta));
        double y = magnitude * Math.sin(Math.toRadians(theta));

        double v = driveTurn * (WHEEL_BASE / Math.hypot(WHEEL_BASE, TRACK_WIDTH));


        double frontY = y + v;
        double backY = y - v;

        double v1 = driveTurn * (TRACK_WIDTH / Math.hypot(WHEEL_BASE, TRACK_WIDTH));

        double leftX = x - v1;
        double rightX = x + v1;

        frontLeft.setTargetAngle(Math.toDegrees(Math.atan2(frontY, leftX)));
        frontRight.setTargetAngle(Math.toDegrees(Math.atan2(frontY, rightX)));
        backLeft.setTargetAngle(Math.toDegrees(Math.atan2(backY, leftX)));
        backRight.setTargetAngle(Math.toDegrees(Math.atan2(backY, rightX)));

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();

        frontLeftPower = Math.hypot(leftX, frontY);
        frontRightPower = Math.hypot(rightX, frontY);
        backRightPower = Math.hypot(rightX, backY);
        backLeftPower = Math.hypot(leftX, backY);
    }



    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower) {

        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(frontLeftPower, Math.max(frontRightPower, Math.max(backLeftPower, backRightPower)));

        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(movementPower * frontLeftPower);
        frontRight.setPower(movementPower * frontRightPower);
        backLeft.setPower(movementPower * backLeftPower);
        backRight.setPower(movementPower * backRightPower);


    }

    @Override
    public String getTelemetry() {
        return "";
    }
}
