package org.firstinspires.ftc.teamcode.library.teleopDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriverOrientedControl {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    public BNO055IMU imu;


    double driveTurn;
    double driveX;
    double driveY;
    double gamepadPower;
    double gamepadTheta;
    double robotTheta;
    double theta;
    double gamepadXControl;
    double gamepadYControl;

    double sin;
    double cos;
    double maxMovement;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;


    public DriverOrientedControl(BNO055IMU imu, DcMotor frontLeft, DcMotor frontRight,
                                 DcMotor backRight, DcMotor backLeft){
        this.imu = imu;

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }


    public void drive(Gamepad gamepad2, double movementPower){

        //might have to make this negative
        driveTurn = -gamepad2.right_stick_x;



        driveX = gamepad2.left_stick_x;
        driveY = -gamepad2.left_stick_y;
        gamepadPower = Range.clip(Math.hypot(driveX, driveY), 0, 1);
        gamepadTheta = Math.toDegrees(Math.atan2(driveY, driveX));

        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        robotTheta = getAngle();
        //gives us the angle our robot is at
        theta = gamepadTheta - robotTheta;


        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        gamepadXControl = Math.cos(Math.toRadians(theta)) * gamepadPower;
        //by finding the adjacent side, we can get our needed x value to power our motors
        gamepadYControl = Math.sin(Math.toRadians(theta)) * gamepadPower;
        //by finding the opposite side, we can get our needed y value to power our motors


//        frontRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl)  - gamepadXControl* Math.abs(gamepadXControl)  + driveTurn));
//        backRight.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn));
//        frontLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl* Math.abs(gamepadXControl) + driveTurn));
//        backLeft.setPower(power*(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn));

        frontRight.setPower(movementPower*(gamepadYControl * Math.abs(gamepadYControl)  - gamepadXControl* Math.abs(gamepadXControl)  + driveTurn));
        backRight.setPower(movementPower*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn));
        frontLeft.setPower(movementPower*(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl* Math.abs(gamepadXControl) - driveTurn));
        backLeft.setPower(movementPower*(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn));


    }

    //run but hopefully better
    public void drive2(Gamepad gamepad2, double movementPower){
        //gamepad input (range -1 to 1)
        driveTurn = gamepad2.right_stick_x;
        driveX = gamepad2.left_stick_x;
        driveY = -gamepad2.left_stick_y;

        //gamepad and robot angles
        gamepadTheta = Math.toDegrees(Math.atan2(driveY, driveX));
        robotTheta = getAngle();

        //power of movement hypotenuse
        gamepadPower = Math.hypot(driveX, driveY);

        //angle of wheels (45 deg for mecanum wheel adjustment)
        theta = gamepadTheta - robotTheta - 45;

        //sin and cos of robot movement
        sin = Math.sin(Math.toRadians(theta));
        cos = Math.cos(Math.toRadians(theta));
        maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

        //determines powers and scales based on max movement
        frontLeftPower = (gamepadPower * cos / maxMovement + driveTurn);
        frontRightPower = (gamepadPower * sin / maxMovement - driveTurn);
        backLeftPower = (gamepadPower * sin / maxMovement + driveTurn);
        backRightPower = (gamepadPower * cos / maxMovement - driveTurn);

        //scales if -1> powers >1
        if(gamepadPower + Math.abs(driveTurn)>1){
            frontLeftPower /= gamepadPower + driveTurn;
            frontRightPower /= gamepadPower + driveTurn;
            backLeftPower /= gamepadPower + driveTurn;
            backRightPower /= gamepadPower + driveTurn;
        }

        //sets powers scaled to desired speed
        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);

    }



    //gets angle from imu
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }



}