package org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.util.AnalogEncoder;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Swerve implements Drivetrain {

    private ModuleV2 frontLeft, frontRight, backRight, backLeft;

    private final double TRACK_WIDTH = 10;
    private final double WHEEL_BASE = 10;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public final double voltageConstant = 13.27;

    public Swerve(HardwareMap hwMap){

        DcMotor leftFront = hwMap.get(DcMotor.class, "lf");
        DcMotor rightFront = hwMap.get(DcMotor.class, "rf");
        DcMotor leftBack = hwMap.get(DcMotor.class, "lr");
        DcMotor rightBack = hwMap.get(DcMotor.class, "rr");

        CRServo leftFrontPivot = hwMap.get(CRServo.class, "lfPivot");
        CRServo rightFrontPivot = hwMap.get(CRServo.class, "rfPivot");
        CRServo leftBackPivot = hwMap.get(CRServo.class, "lrPivot");
        CRServo rightBackPivot = hwMap.get(CRServo.class, "rrPivot");

        leftFrontPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        AnalogEncoder leftFrontEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "lfEnc"));
        AnalogEncoder rightFrontEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "rfEnc"));
        AnalogEncoder leftBackEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "lrEnc"));
        AnalogEncoder rightBackEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "rrEnc"));

        frontLeft = new ModuleV2(leftFront, leftFrontPivot, leftFrontEnc);
        frontRight = new ModuleV2(rightFront, rightFrontPivot, rightFrontEnc);
        backLeft = new ModuleV2(leftBack, leftBackPivot, leftBackEnc);
        backRight = new ModuleV2(rightBack, rightBackPivot, rightBackEnc);
    }


    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower) {
        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
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

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

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
    public void drive(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftPower *= movementPower;
        frontRightPower *= movementPower;
        backLeftPower *= movementPower;
        backRightPower *= movementPower;

        scaleByVoltage(voltage);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


    }

    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftPower *= movementPower;
        frontRightPower *= movementPower;
        backLeftPower *= movementPower;
        backRightPower *= movementPower;

        scaleByVoltage(voltage);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


    }

    public void scaleByVoltage(double voltage){
        frontLeftPower /= voltage;
        frontRightPower /= voltage;
        backLeftPower /= voltage;
        backRightPower /= voltage;


        frontLeftPower *= voltageConstant;
        frontRightPower *= voltageConstant;
        backLeftPower *= voltageConstant;
        backRightPower *= voltageConstant;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

    }

    @Override
    public String getTelemetry() {
        return "Front Left: " + frontLeft.getModuleAngle() + "\n"
            + "Front Right: " + frontRight.getModuleAngle() + "\n"
                + "Back Left: " + backLeft.getModuleAngle() + "\n"
                + "Back Right: " + backRight.getModuleAngle() ;
    }
}
