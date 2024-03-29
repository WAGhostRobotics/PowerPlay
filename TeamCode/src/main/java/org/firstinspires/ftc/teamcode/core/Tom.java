package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.Latch;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.Pivot;
import org.firstinspires.ftc.teamcode.component.Webcam;

public class Tom {
    public static HardwareMap hardwareMap;

    public static Motor frontLeft;
    public static Motor frontRight;
    public static Motor backLeft;
    public static Motor backRight;

    public static Webcam webcam;
    public static RevIMU imu;

    public static IntakeSlides intake;
    public static OuttakeSlides outtake;
    public static Claw claw;
    public static Arm arm;
    public static Latch latch;



    public static void init(HardwareMap hwMap, boolean initTeleOp) {
        // Assign HardwareMap
        hardwareMap = hwMap;

        imu = new RevIMU(hwMap);
        imu.init();

        latch = new Latch();
        latch.init(hwMap);


        webcam = new Webcam();
        webcam.init(hwMap);

        intake = new IntakeSlides();


        outtake = new OuttakeSlides();


        claw = new Claw();
        claw.init(hwMap);

        arm = new Arm();




        if(initTeleOp){

            frontLeft = new Motor(hwMap, "lf");
            frontRight = new Motor(hwMap, "rf");
            backLeft = new Motor(hwMap, "lr");
            backRight = new Motor(hwMap, "rr");
//
//            frontLeft.setInverted(true);
//            frontRight.setInverted(true);
//            backLeft.setInverted(true);
//            backRight.setInverted(true);


            frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


            arm.init(hwMap, true);
            intake.init(hwMap, true);
            outtake.init(hwMap, true);
        }else{


            arm.init(hwMap, false);
            intake.init(hwMap, false);
            outtake.init(hwMap, false);
//            System.exit(0);

            imu = new RevIMU(hardwareMap, "imu");
            imu.init();
        }
    }


    public static void initIMU(){
        imu = new RevIMU(hardwareMap, "imu");
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
