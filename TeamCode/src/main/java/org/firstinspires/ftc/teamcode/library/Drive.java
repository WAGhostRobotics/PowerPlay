package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private double sin;
    private double cos;
    private double maxMovement;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    String telemetry = "";


    public Drive(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft = hardwareMap.get(DcMotorEx.class, "lr");
        backRight = hardwareMap.get(DcMotorEx.class, "rr");

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower){

        theta -= 45;

        //sin and cos of robot movement
        sin = Math.sin(Math.toRadians(theta));
        cos = Math.cos(Math.toRadians(theta));
        maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

        //determines powers and scales based on max movement
        frontLeftPower = (magnitude * cos / maxMovement - driveTurn);
        frontRightPower = (magnitude * sin / maxMovement + driveTurn);
        backLeftPower = (magnitude * sin / maxMovement - driveTurn);
        backRightPower = (magnitude * cos / maxMovement + driveTurn);

        //scales if -1 < powers < 1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + driveTurn;
            frontRightPower /= magnitude + driveTurn;
            backLeftPower /= magnitude + driveTurn;
            backRightPower /= magnitude + driveTurn;
        }

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);


        telemetry = "" + frontLeftPower + " \n" + frontRightPower + " \n" + backLeftPower + " \n" + backRightPower;

    }

    public void drive(double magnitude, double theta, double driveTurn, double movementPower){

        theta -= 45;

        //sin and cos of robot movement
        sin = Math.sin(Math.toRadians(theta));
        cos = Math.cos(Math.toRadians(theta));
        maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

        //determines powers and scales based on max movement
        frontLeftPower = (magnitude * cos / maxMovement + driveTurn);
        frontRightPower = (magnitude * sin / maxMovement - driveTurn);
        backLeftPower = (magnitude * sin / maxMovement + driveTurn);
        backRightPower = (magnitude * cos / maxMovement - driveTurn);

        //scales if -1 < powers < 1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + driveTurn;
            frontRightPower /= magnitude + driveTurn;
            backLeftPower /= magnitude + driveTurn;
            backRightPower /= magnitude + driveTurn;
        }


        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);
    }
}
