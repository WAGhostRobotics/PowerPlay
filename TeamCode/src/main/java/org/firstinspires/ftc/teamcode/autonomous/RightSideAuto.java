package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    int conePlaced = 0;

    public int position = LinearSlidesArm.TurnValue.GROUND.getTicks();



    enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_6,
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

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build();


//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .back(10)
//                .build();
//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .lineToSplineHeading(new Pose2d(47, 0, Math.toRadians(270)))
//                .splineTo(new Vector2d(47, 12), Math.toRadians(270))
//                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(10)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(10)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(39, 23, Math.toRadians(180)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(39, -23, Math.toRadians(270)))
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
        int distance = 0;

        if(location == Webcam.Location.ONE){
            distance = 30;
        }else if(location == Webcam.Location.TWO){
            distance = 20;
        } else if(location == Webcam.Location.THREE){
            distance = 10;
        }

        Trajectory traj6 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(39, distance, Math.toRadians(270)))
                .build();

        currentState = State.TRAJECTORY_1;




        drive.followTrajectoryAsync(traj1);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_1:

                    if (!drive.isBusy()) {
                        position = LinearSlidesArm.TurnValue.TOP.getTicks();
                        time.reset();

                        if(time.milliseconds()>1500){
                            break;
//                            currentState = State.TRAJECTORY_2;
//                            drive.followTrajectoryAsync(traj2);
                        }
                    }

                    break;
                case TRAJECTORY_2:

                    if (!drive.isBusy()) {
                        Jerry.intake.out();
                        sleep(1000);
                        Jerry.intake.stop();

                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(traj3);
                    }

                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        position = LinearSlidesArm.TurnValue.CONES.getTicks();

                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectoryAsync(traj4);
                    }

                    break;
                case TRAJECTORY_4:
                    if (!drive.isBusy()) {
                        Jerry.intake.in();
                        if(conePlaced == 0){
                            position -= 50;
                        }
                        time.reset();
                        if(time.milliseconds() > 1000){
                            Jerry.intake.stop();
                            position = LinearSlidesArm.TurnValue.CONES.getTicks();
                        }
                        time.reset();
                        if(time.milliseconds() > 1000 && conePlaced < 5){

                            conePlaced++;
                            currentState = State.TRAJECTORY_1;
                            drive.followTrajectoryAsync(traj5);
                        }else{
                            currentState = State.TRAJECTORY_6;
                            drive.followTrajectoryAsync(traj6);
                        }
                    }

                    break;
                case TRAJECTORY_6:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:

                    break;
            }

            drive.update();

            if(!Jerry.slides.isBusy()){
                Jerry.slides.stopArm();
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
