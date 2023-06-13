package org.firstinspires.ftc.teamcode.library.drivetrain.SwerveDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;

public class Swerve implements Drivetrain {

    private ModuleV2 frontLeft, frontRight, backRight, backLeft;

    private final double TRACK_WIDTH = 10;
    private final double WHEEL_BASE = 10;

    public Swerve(HardwareMap hwMap, boolean reset){
        frontLeft = new ModuleV2(hwMap, "lf", "lfPivot", "lfEnc", reset);
        frontRight = new ModuleV2(hwMap, "rf", "rfPivot", "rfEnc", reset);
        backRight = new ModuleV2(hwMap, "rr", "rrPivot", "rrEnc", reset);
        backLeft = new ModuleV2(hwMap, "lr", "lrPivot", "lrEnc", reset);
    }


    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower) {
        double x = magnitude * Math.cos(Math.toRadians(theta));
        double y = magnitude * Math.sin(Math.toRadians(theta));

        x = x/Math.hypot(x, y) * movementPower;
        y = y/Math.hypot(x, y) * movementPower;

        double v = driveTurn * (WHEEL_BASE / Math.hypot(WHEEL_BASE, TRACK_WIDTH));


        double front = x + v;
        double back = x - v;

        double v1 = driveTurn * (TRACK_WIDTH / Math.hypot(WHEEL_BASE, TRACK_WIDTH));

        double left = y - v1;
        double right = y + v1;

        double frontLeftPower = Math.hypot(left, front);
        double frontRightPower = Math.hypot(right, front);
        double backRightPower = Math.hypot(right, back);
        double backLeftPower = Math.hypot(left, back);

        double max = Math.max(frontLeftPower, Math.max(frontRightPower, Math.max(backLeftPower, backRightPower)));

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        frontLeft.setTargetAngle(Math.toDegrees(Math.atan2(front, left)));
        frontRight.setTargetAngle(Math.toDegrees(Math.atan2(front, right)));
        backLeft.setTargetAngle(Math.toDegrees(Math.atan2(back, left)));
        backRight.setTargetAngle(Math.toDegrees(Math.atan2(back, right)));

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();

    }



    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower) {
        double x = magnitude * Math.sin(Math.toRadians(theta));
        double y = magnitude * Math.cos(Math.toRadians(theta));

        x = x/Math.hypot(x, y) * movementPower;
        y = y/Math.hypot(x, y) * movementPower;

        double v = driveTurn * (WHEEL_BASE / Math.hypot(WHEEL_BASE, TRACK_WIDTH));


        double front = x - v;
        double back = x + v;

        double v1 = driveTurn * (TRACK_WIDTH / Math.hypot(WHEEL_BASE, TRACK_WIDTH));

        double left = y + v1;
        double right = y - v1;

        double frontLeftPower = Math.hypot(left, front);
        double frontRightPower = Math.hypot(right, front);
        double backRightPower = Math.hypot(right, back);
        double backLeftPower = Math.hypot(left, back);

        double max = Math.max(frontLeftPower, Math.max(frontRightPower, Math.max(backLeftPower, backRightPower)));

        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        frontLeft.setTargetAngle(Math.toDegrees(Math.atan2(left, front)));
        frontRight.setTargetAngle(Math.toDegrees(Math.atan2(right, front)));
        backLeft.setTargetAngle(Math.toDegrees(Math.atan2(left, back)));
        backRight.setTargetAngle(Math.toDegrees(Math.atan2(right, back)));

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    @Override
    public String getTelemetry() {
        return "";
    }
}
