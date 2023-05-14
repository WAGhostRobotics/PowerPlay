package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class WonkyDrive {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    private Encoder xEncoder;
    private Encoder yEncoder;

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

    double velocity;

    ElapsedTime time;

    double radius;

    public static double WHEEL_RADIUS = 96.0/2/25.4 ; // in
    public static double GEAR_RATIO = 1 ; // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 537.6;

    public final double THE_HOLY_CONSTANT = 0.01;
    
    double ac;


    public WonkyDrive(BNO055IMU imu, DcMotor frontLeft, DcMotor frontRight,
                      DcMotor backRight, DcMotor backLeft, Encoder xEncoder, Encoder yEncoder){
        this.imu = imu;

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;


//        xEncoder.setDirection(Encoder.Direction.REVERSE);
//        yEncoder.setDirection(Encoder.Direction.REVERSE);

        y1 = 0;
        y2 = 0;
        lasty = yEncoder.getCurrentPosition();
        lasty1 = 0;
        lastx = xEncoder.getCurrentPosition();

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

        //power of movement hypotenuse
        gamepadMagnitude = Math.hypot(driveX, driveY);

        //angle of wheels (45 deg for mecanum wheel adjustment)
        theta = gamepadTheta - robotTheta;


        //movement vector is magnitude: gameMagnitude direct: theta
        if(!Double.isNaN(y1)&&!Double.isNaN(y2)){
            radius = Math.pow((1+Math.pow(y1,2)), 1.5)/y2;
            ac = Math.pow(velocity, 2)/radius;
            theta -= Math.toDegrees(Math.atan2(ac*THE_HOLY_CONSTANT, gamepadMagnitude));
            gamepadMagnitude = Math.hypot(gamepadMagnitude, ac*THE_HOLY_CONSTANT);

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

    public void updateValues(){

        if((getX()-lastx) == 0){
            y1 = Double.NaN;
            y2 = Double.NaN;
        }else{
            y1 = (getY()-lasty)/(getX()-lastx);
            y2 = (y1-lasty1)/(getX()-lastx);
        }



        velocity = Math.sqrt(
                Math.pow(((getX()-lastx)/time.seconds()), 2) + Math.pow(((getY()-lasty)/time.seconds()), 2)
        );   //pythagorean theorme :)

        lasty1 = y1;
        lastx = getX();
        lasty = getY();

        time.reset();
    }

    public double getX(){
        return encoderTicksToInches(xEncoder.getCurrentPosition());
    }

    public double getY(){
        return encoderTicksToInches(yEncoder.getCurrentPosition());
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    //gets angle from imu
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}