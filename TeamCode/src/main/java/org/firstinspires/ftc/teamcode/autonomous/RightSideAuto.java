package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Jerry;
import org.firstinspires.ftc.teamcode.library.DriveStyle;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Right Side", group = "competition")
public class RightSideAuto extends LinearOpMode {

    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    final double POWER = 0.5;




    @Override
    public void runOpMode() {
        // feedback after OpMode started
        telemetry.addData("Status", "Initializing...");
        telemetry.update();



        Jerry.init(hardwareMap, false);

        // updates feedback after initialization finished
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //telemetry of the vision data
        while (!isStarted() && !isStopRequested()) {
            Jerry.webcam.scanForTags();
            location = Jerry.webcam.getLocation();
            tagOfInterest = Jerry.webcam.getTagOfInterest();


            //telemetry for the position
            if (location != null && tagOfInterest != null){
                telemetry.addData("Location", location);
                telemetry.addData("Tag ID", tagOfInterest.id);
                telemetry.addLine("\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }


        }
        Jerry.webcam.stopStreaming();


        if(location == Webcam.Location.ONE){

            DriveStyle.MecanumArcade(Jerry.driveMotors, POWER, 1, 0, 0);
            sleep(1100);
            DriveStyle.MecanumArcade(Jerry.driveMotors, 0, 0, 0, 0);
        }else if (location == Webcam.Location.TWO){

        }else if (location == Webcam.Location.THREE){
            DriveStyle.MecanumArcade(Jerry.driveMotors, POWER, -1, 0, 0);
            sleep(1100);
            DriveStyle.MecanumArcade(Jerry.driveMotors, 0, 0, 0, 0);
        }

        straighten(0, 0.2);
        sleep(500);
        DriveStyle.MecanumArcade(Jerry.driveMotors, POWER, 0, -1, 0);
        sleep(1500);
        DriveStyle.MecanumArcade(Jerry.driveMotors, 0, 0, 0, 0);



    }

    public void straighten(int heading, double power) {
// Left positive
//right negative
        switch(heading) {
            case 0:
                while (Math.abs(Jerry.imu.getHeading()) > 3) {//5
                    if (Jerry.imu.getHeading() < 0) {
                        DriveStyle.MecanumArcade(Jerry.driveMotors, -power, 0, 0, 1);
                    } else {
                        DriveStyle.MecanumArcade(Jerry.driveMotors, power, 1, 0, 1);
                    }
                }
                break;

        }

        DriveStyle.MecanumArcade(Jerry.driveMotors, 0, 0, 0 ,0);
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
