package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Left Side", group = "competition")
public class RightSideAuto extends LinearOpMode {
    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    LeftSideAuto.State currentState = LeftSideAuto.State.IDLE;

    double power = 1;
    int position = 0;

    enum State {
        IDLE
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // feedback after OpMode started
        telemetry.addData("Status", "Initializing...");
        telemetry.update();


        Tom.init(hardwareMap, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));


        // updates feedback after initialization finished
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //telemetry of the vision data
        while (!isStarted() && !isStopRequested()) {
            Tom.webcam.scanForTags();
            location = Tom.webcam.getLocation();
            tagOfInterest = Tom.webcam.getTagOfInterest();


            //telemetry for the position
            if (location != null && tagOfInterest != null) {
                telemetry.addData("Location", location);
                telemetry.addData("Tag ID", tagOfInterest.id);
                telemetry.addLine("\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }


        }
    }


    private void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}