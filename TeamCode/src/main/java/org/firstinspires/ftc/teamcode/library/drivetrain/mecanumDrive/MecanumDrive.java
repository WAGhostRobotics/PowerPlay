package org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;

public class MecanumDrive implements Drivetrain {

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


    public MecanumDrive(HardwareMap hardwareMap){
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

    @Override
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

        //scales
        frontLeftPower /= magnitude + Math.abs(driveTurn);
        frontRightPower /= magnitude + Math.abs(driveTurn);
        backLeftPower /= magnitude + Math.abs(driveTurn);
        backRightPower /= magnitude + Math.abs(driveTurn);


        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);


        telemetry = "" + frontLeftPower + " \n" + frontRightPower + " \n" + backLeftPower + " \n" + backRightPower;

    }

    @Override
    public String getTelemetry(){
        return telemetry;
    }

    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower){

        theta += 45;

        //sin and cos of robot movement
        sin = Math.sin(Math.toRadians(theta));
        cos = Math.cos(Math.toRadians(theta));
        maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

        //determines powers and scales based on max movement
        frontLeftPower = (magnitude * cos / maxMovement + driveTurn);
        frontRightPower = (magnitude * sin / maxMovement - driveTurn);
        backLeftPower = (magnitude * sin / maxMovement + driveTurn);
        backRightPower = (magnitude * cos / maxMovement - driveTurn);

        //scales if -1> powers >1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + Math.abs(driveTurn);
            frontRightPower /= magnitude + Math.abs(driveTurn);
            backLeftPower /= magnitude + Math.abs(driveTurn);
            backRightPower /= magnitude + Math.abs(driveTurn);
        }


        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);
    }
}
