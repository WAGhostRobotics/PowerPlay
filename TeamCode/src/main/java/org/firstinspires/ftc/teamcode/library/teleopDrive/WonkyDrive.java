package org.firstinspires.ftc.teamcode.library.teleopDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;

@Config
public class WonkyDrive {

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


    public static double theHolyConstant = 0.001; //0.01
    public static double rotationalDriftConstant = 0.0027; //0.002

    double ac;

    public Localizer localizer;
    public Drivetrain drive;



    public static PIDController headingController = new PIDController(0.017, 0.0, 0.0);

    public static double p = headingController.getP(), i = headingController.getI(), d = headingController.getD();
//    PIDController headingController = new PIDController(0, 0, 0);


    public WonkyDrive(HardwareMap hardwareMap, Localizer localizer, Drivetrain drive){



        imu = hardwareMap.get(BNO055IMU.class, "imu");



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
        driveTurn = power * Math.pow(-gamepad2.right_stick_x, 3);
        driveY = power * Math.pow(-gamepad2.left_stick_x, 3);
        driveX = power * Math.pow(-gamepad2.left_stick_y, 3);

        //gamepad and robot angles
        gamepadTheta = Math.toDegrees(Math.atan2(driveY, driveX));
        robotTheta = getCurrentHeading();

        currentHeading = getCurrentHeading();


        double headingError = getHeadingError();

//        if(driveTurn != 0 || (driveX == 0 && driveY == 0)){
        if(driveTurn != 0){
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
            theta -= Math.toDegrees(Math.atan2( ac* theHolyConstant, gamepadMagnitude));
            gamepadMagnitude = Math.hypot(gamepadMagnitude, ac* theHolyConstant);

        }else{
            ac = 0;
        }

        drive.drive(gamepadMagnitude, theta, driveTurn, 1);

    }

    public double getHeadingError() {
        double headingError = lastHeading - currentHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }

        return headingError;
    }

    public double getAc(){
        return ac;
    }

    public String getTelemetry(){
        return "Angular velocity: " + imu.getAngularVelocity().xRotationRate +
                "\nDrift Angle: " +  Math.signum(imu.getAngularVelocity().xRotationRate) * 0.5 * Math.pow(imu.getAngularVelocity().xRotationRate, 2) * 0.002;

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

        double omega = imu.getAngularVelocity().xRotationRate;

        lastHeading = normalizeDegrees(getCurrentHeading() + Math.signum(omega) * 0.5 * Math.pow(omega, 2) * rotationalDriftConstant);
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