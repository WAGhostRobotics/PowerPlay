package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Jerry;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous(name = "Pole Detection Test", group = "competition")
public class PoleDetectionTest extends LinearOpMode {


    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();




        Jerry.init(hardwareMap, false);


        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Pole detected: ", Jerry.webcam.poleInPlace());
            telemetry.addData("Percentage Occupied: ", Jerry.webcam.getPercentageOfPole());

            telemetry.update();

        }
        Jerry.webcam.stopStreaming();




    }









}
