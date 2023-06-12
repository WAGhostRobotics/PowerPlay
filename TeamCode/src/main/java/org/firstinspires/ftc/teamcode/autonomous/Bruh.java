package org.firstinspires.ftc.teamcode.autonomous;



import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Tom;
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

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();

        waitForStart();

        motionPlanner.startTrajectory(new Bezier(new Point(0, 0), new Point(40, 0)), 0);

        while (!isStopRequested()) {



            motionPlanner.update();


            telemetry.addLine("" + motionPlanner.getTelemetry());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }
    }
}

