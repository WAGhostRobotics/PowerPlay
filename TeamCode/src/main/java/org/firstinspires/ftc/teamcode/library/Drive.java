package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    private double sin;
    private double cos;
    private double maxMovement;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;


    public Drive(DcMotor frontLeft, DcMotor frontRight,
                 DcMotor backRight, DcMotor backLeft){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;


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

        double max = Math.max(frontLeftPower, Math.max(frontRightPower, Math.max(backLeftPower, backRightPower)));

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);
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
