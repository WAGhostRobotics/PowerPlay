package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.component.IMU;
import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.Webcam;

import java.util.ArrayList;

public class Jerry {
    public static HardwareMap hardwareMap;

    public static Motor frontLeft;
    public static Motor frontRight;
    public static Motor backLeft;
    public static Motor backRight;

    public static Webcam webcam;

    public static Intake intake;



    public static RevIMU imu;

    public static void init(HardwareMap hwMap, boolean initTeleOp) {
        // Assign HardwareMap
        hardwareMap = hwMap;

        imu = new RevIMU(hwMap);
        imu.init();

        webcam = new Webcam();
        webcam.init(hwMap);

        intake = new Intake();
        intake.init(hwMap);

        if(initTeleOp){

//            frontLeft = new Motor(hwMap, "lf");
//            frontRight = new Motor(hwMap, "lr");
//            backLeft = new Motor(hwMap, "rf");
//            backRight = new Motor(hwMap, "rr");
//
//            frontLeft.setInverted(true);
//            frontRight.setInverted(true);
//            backLeft.setInverted(true);
//            backRight.setInverted(true);
//
//            frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        }else{


        }





    }


    public static void initIMU(){
        imu = new RevIMU(hardwareMap);
        imu.init();

    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
