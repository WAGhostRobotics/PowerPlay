package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.component.LinearSlidesArm;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Jerry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.library.DriveStyle;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Right Side", group = "competition")
public class RightSideAuto extends LinearOpMode {

    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    State currentState = State.IDLE;

    final double POWER = 0.5;

    public int position = LinearSlidesArm.TurnValue.GROUND.getTicks();



    enum State {
        TRAJECTORY_1,
        IDLE
    }

    @Override
    public void runOpMode() {
        // feedback after OpMode started
        telemetry.addData("Status", "Initializing...");
        telemetry.update();



        Jerry.init(hardwareMap, false);
        Jerry.initIMU();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(39, -31, Math.toRadians(270)))
                .build();

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

        while (opModeIsActive()) {
            switch (currentState) {
                case TRAJECTORY_1:
                    position = LinearSlidesArm.TurnValue.GROUND.getTicks();

                    break;
                case IDLE:

                    break;
            }

            drive.update();

            if(!Jerry.slides.isBusy()){
                Jerry.slides.stopArm();
            }




            if(location == Webcam.Location.ONE){

            }else if (location == Webcam.Location.TWO){

            }else if (location == Webcam.Location.THREE){

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
