package org.firstinspires.ftc.teamcode.autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.Drive;
import org.firstinspires.ftc.teamcode.library.Localizer;
import org.firstinspires.ftc.teamcode.library.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.math.Bezier;
import org.firstinspires.ftc.teamcode.library.math.Point;


@Autonomous(name = "Bruh GG", group = "competition")
public class Bruh extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);




        Localizer localizer = new Localizer(hardwareMap);
        Drive drive = new Drive(hardwareMap);

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer);


        waitForStart();

        motionPlanner.startTrajectory(new Bezier(new Point(0, 0), new Point(10, 40), new Point(20, 0)), 0);

        while (!isStopRequested()) {



            motionPlanner.update();


            telemetry.addLine("" + motionPlanner.getTelemetry());
            telemetry.update();
        }
    }
}

