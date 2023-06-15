package org.firstinspires.ftc.teamcode.autonomous;



import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.MergedBezier;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;


@Autonomous(name = "Bruh GG", group = "competition")
public class Bruh extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);




        Localizer localizer = new Localizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();


        waitForStart();

//        ElapsedTime wait = new ElapsedTime();

//        while(wait.seconds()<7){
////            telemetry.addLine("" + motionPlanner.getTelemetry());
////            telemetry.update();
//
//
//        }

//        motionPlanner.startTrajectory(new Bezier(90, new Point(0, 0), new Point(40, 0)));

            motionPlanner.startTrajectory(new Bezier(
                    new Point(0,0),
                    new Point(45, 0),
                    new Point(12, 25),
                    new Point(45, 25)
            ));

        while (opModeIsActive() && !isStopRequested()) {





            motionPlanner.update();

            if(motionPlanner.isFinished()){
                telemetry.addData("x", localizer.getX());
                telemetry.addData("y", localizer.getY());
                telemetry.addData("heading", localizer.getHeading(Localizer.Angle.DEGREES));
                telemetry.update();
            }else{
                telemetry.addLine("" + motionPlanner.getTelemetry());
                telemetry.update();
            }




            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

    }
}

