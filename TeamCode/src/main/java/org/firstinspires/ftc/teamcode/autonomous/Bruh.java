package org.firstinspires.ftc.teamcode.autonomous;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.library.Drive;
import org.firstinspires.ftc.teamcode.library.Localizer;
import org.firstinspires.ftc.teamcode.library.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.math.Bezier;
import org.firstinspires.ftc.teamcode.library.math.Point;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Autonomous(name = "Bruh GG", group = "competition")
public class Bruh extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "lf");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rf");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "lr");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "rr");

        Drive drive = new Drive(frontLeft, frontRight, backRight, backLeft);


        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        rightEncoder.setDirection(Encoder.Direction.REVERSE);


        Localizer localizer = new Localizer(rightEncoder, frontEncoder, leftEncoder);


        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer);


        waitForStart();

        motionPlanner.startTrajectory(new Bezier(new Point(0, 0), new Point(20, 0)), 0);

        while (!isStopRequested()) {


            motionPlanner.update();


            telemetry.addLine("Good luck");
            telemetry.update();
        }
    }
}

