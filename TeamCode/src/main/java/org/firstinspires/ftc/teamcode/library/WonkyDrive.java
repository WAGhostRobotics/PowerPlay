package org.firstinspires.ftc.teamcode.library;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class WonkyDrive {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;


    public BNO055IMU imu;


    double driveTurn;
    double driveX;
    double driveY;
    double gamepadMagnitude;
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

    double y1;
    double y2;

    double lasty;
    double lasty1;
    double lastx;
    double lastHeading;

    double currentY;
    double currentX;



    double velocity;

    ElapsedTime time;

    double radius;

    public String stuff = "";


    public final double THE_HOLY_CONSTANT = 0.0003; //0.01

    double ac;

    public Localizer localizer;

    PController headingController = new PController(1.0/90.0);


    public WonkyDrive(HardwareMap hardwareMap, Localizer localizer){



        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft = hardwareMap.get(DcMotorEx.class, "lr");
        backRight = hardwareMap.get(DcMotorEx.class, "rr");

//            frontLeft.setInverted(true);
//            frontRight.setInverted(true);
//            backLeft.setInverted(true);
//            backRight.setInverted(true);

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



        this.localizer = localizer;


//        xEncoder.setDirection(Encoder.Direction.REVERSE);
//        yEncoder.setDirection(Encoder.Direction.REVERSE);

        y1 = 0;
        y2 = 0;
        lasty = 0;
        lasty1 = 0;
        lastx = 0;

        time = new ElapsedTime();
    }


    public void drive(Gamepad gamepad2, double movementPower){


        updateValues();



        //gamepad input (range -1 to 1)
        driveTurn = gamepad2.right_stick_x;
        driveX = gamepad2.left_stick_x;
        driveY = -gamepad2.left_stick_y;

        //gamepad and robot angles
        gamepadTheta = Math.toDegrees(Math.atan2(driveY, driveX));
        robotTheta = getAngle();

        if(driveTurn != 0){
            //power of movement hypotenuse
            gamepadMagnitude = Range.clip(Math.hypot(driveX, driveY), 0, 1);

            //angle of wheels (45 deg for mecanum wheel adjustment)
            theta = gamepadTheta - robotTheta;


            //movement vector is magnitude: gameMagnitude direct: theta
            if(!Double.isNaN(y1)&&!Double.isNaN(y2)&& gamepadMagnitude != 0){
                radius = Math.pow((1+Math.pow(y1,2)), 1.5)/y2;
                ac = Math.pow(velocity, 2)/radius;
                theta -= Math.toDegrees(Math.atan2( ac*THE_HOLY_CONSTANT, gamepadMagnitude));
                gamepadMagnitude = Math.hypot(gamepadMagnitude, ac*THE_HOLY_CONSTANT);

            }else{
                ac = 0;
            }

            theta -= 45;

            //sin and cos of robot movement
            sin = Math.sin(Math.toRadians(theta));
            cos = Math.cos(Math.toRadians(theta));
            maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

            //determines powers and scales based on max movement
            frontLeftPower = (gamepadMagnitude * cos / maxMovement + driveTurn);
            frontRightPower = (gamepadMagnitude * sin / maxMovement - driveTurn);
            backLeftPower = (gamepadMagnitude * sin / maxMovement + driveTurn);
            backRightPower = (gamepadMagnitude * cos / maxMovement - driveTurn);

            //scales if -1> powers >1
            if(gamepadMagnitude + Math.abs(driveTurn)>1){
                frontLeftPower /= gamepadMagnitude + driveTurn;
                frontRightPower /= gamepadMagnitude + driveTurn;
                backLeftPower /= gamepadMagnitude + driveTurn;
                backRightPower /= gamepadMagnitude + driveTurn;
            }

            //sets powers scaled to desired speed
            frontLeft.setPower(movementPower*frontLeftPower);
            frontRight.setPower(movementPower*frontRightPower);
            backLeft.setPower(movementPower*backLeftPower);
            backRight.setPower(movementPower*backRightPower);
        }else{
            //power of movement hypotenuse

            driveTurn = headingController.calculate(getAngle(), lastHeading);
            gamepadMagnitude = Range.clip(Math.hypot(driveX, driveY), 0, 1);

            //angle of wheels (45 deg for mecanum wheel adjustment)
            theta = gamepadTheta - robotTheta;


            //movement vector is magnitude: gameMagnitude direct: theta
            if(!Double.isNaN(y1)&&!Double.isNaN(y2)&& gamepadMagnitude != 0){
                radius = Math.pow((1+Math.pow(y1,2)), 1.5)/y2;
                ac = Math.pow(velocity, 2)/radius;
                theta -= Math.toDegrees(Math.atan2( ac*THE_HOLY_CONSTANT, gamepadMagnitude));
                gamepadMagnitude = Math.hypot(gamepadMagnitude, ac*THE_HOLY_CONSTANT);

            }else{
                ac = 0;
            }

            theta -= 45;

            //sin and cos of robot movement
            sin = Math.sin(Math.toRadians(theta));
            cos = Math.cos(Math.toRadians(theta));
            maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

            //determines powers and scales based on max movement
            frontLeftPower = (gamepadMagnitude * cos / maxMovement + driveTurn);
            frontRightPower = (gamepadMagnitude * sin / maxMovement - driveTurn);
            backLeftPower = (gamepadMagnitude * sin / maxMovement + driveTurn);
            backRightPower = (gamepadMagnitude * cos / maxMovement - driveTurn);

            //scales if -1> powers >1
            if(gamepadMagnitude + Math.abs(driveTurn)>1){
                frontLeftPower /= gamepadMagnitude + driveTurn;
                frontRightPower /= gamepadMagnitude + driveTurn;
                backLeftPower /= gamepadMagnitude + driveTurn;
                backRightPower /= gamepadMagnitude + driveTurn;
            }

            //sets powers scaled to desired speed
            frontLeft.setPower(movementPower*frontLeftPower);
            frontRight.setPower(movementPower*frontRightPower);
            backLeft.setPower(movementPower*backLeftPower);
            backRight.setPower(movementPower*backRightPower);
        }

        stuff = "" + frontLeftPower + " " + frontRightPower + " " + backLeftPower + " " + backRightPower;

    }

    public double getAc(){
        return ac;
    }



    public void updateValues(){

        localizer.update();

        currentX = getX();
        currentY = getY();

        if((currentX-lastx) == 0){
            y1 = Double.NaN;
            y2 = Double.NaN;
        }else{
            y1 = (currentY-lasty)/(currentX-lastx);
            y2 = (y1-lasty1)/(currentX-lastx);
        }


        double t = time.seconds();

        velocity = Math.sqrt(
                Math.pow(((currentX-lastx)/t), 2) + Math.pow(((currentY-lasty)/t), 2)
        );   //pythagorean theorme :)

        lasty1 = y1;
        lastx = currentX;
        lasty = currentY;

        lastHeading = getAngle();

        time.reset();
    }

    public double getX(){
        return localizer.getRawY();
    }

    public double getY(){
        return localizer.getRawX();
    }



    //gets angle from imu
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}