package org.firstinspires.ftc.teamcode.library.teleopDrive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;

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

    double y1;
    double y2;

    double lasty;
    double lasty1;
    double lastx;
    double lastHeading = 0;
    double currentHeading = 0;

    double currentY;
    double currentX;



    double velocity;

    ElapsedTime time;

    double radius;

    public String stuff = "";


    public final double THE_HOLY_CONSTANT = 0.001; //0.01

    double ac;

    public Localizer localizer;
    public Drivetrain drive;

    PIDController headingController = new PIDController(5.0/90.0, 0.01, 0.005);
//    PIDController headingController = new PIDController(0, 0, 0);


    public WonkyDrive(HardwareMap hardwareMap, Localizer localizer, Drivetrain drive){



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
        this.drive = drive;


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

        double power;

        if(gamepad2.right_trigger >= 0.01){
            power = 0.9;
        }else{
            power = movementPower;
        }


        //gamepad input (range -1 to 1)
        driveTurn = power * Math.pow(gamepad2.right_stick_x, 3);
        driveY = power * Math.pow(-gamepad2.left_stick_x, 3);
        driveX = power * Math.pow(-gamepad2.left_stick_y, 3);

        //gamepad and robot angles
        gamepadTheta = Math.toDegrees(Math.atan2(driveY, driveX));
        robotTheta = getCurrentHeading();

        currentHeading = getCurrentHeading();


        double headingError = currentHeading - lastHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }

        if(driveTurn != 0 || (driveX == 0 && driveY == 0)){
            updateHeading();
            headingController.reset();
        }else if (Math.abs(headingError) > 0.5 ){
            driveTurn = headingController.calculate(0, headingError);
        }else{
            headingController.reset();
        }

        gamepadMagnitude = Range.clip(Math.hypot(driveX, driveY), 0, 1);
        theta = gamepadTheta - robotTheta;
        if(!Double.isNaN(y1)&&!Double.isNaN(y2)&& gamepadMagnitude != 0){
            radius = Math.pow((1+Math.pow(y1,2)), 1.5)/y2;
            ac = Math.pow(velocity, 2)/radius;
            theta -= Math.toDegrees(Math.atan2( ac*THE_HOLY_CONSTANT, gamepadMagnitude));
            gamepadMagnitude = Math.hypot(gamepadMagnitude, ac*THE_HOLY_CONSTANT);

        }else{
            ac = 0;
        }

        drive.drive(gamepadMagnitude, theta, driveTurn, 1);

    }

    public double getAc(){
        return ac;
    }

    public String getTelemetry(){
        return "" + getCurrentHeading() + "\n" +  lastHeading;
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



        time.reset();
    }

    public void updateHeading(){
        lastHeading = getCurrentHeading();
    }

    public double getX(){
        return localizer.getRawY();
    }

    public double getY(){
        return localizer.getRawX();
    }



    //gets angle from imu
    public double getCurrentHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}