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
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Right Side", group = "competition")
public class RightSideAuto extends LinearOpMode {

    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    State currentState = State.IDLE;



    double power = 1;

    int conePlaced = 0;


    int position = 0;



    enum State {
        TRAJECTORY_0,
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        GRAB_CONE,
        TRAJECTORY_6,
        IDLE
    }

    @Override
    public void runOpMode() throws InterruptedException{
        // feedback after OpMode started
        telemetry.addData("Status", "Initializing...");
        telemetry.update();



        Jerry.init(hardwareMap, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,0));


        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(51.5, 6, Math.toRadians(270)))
                .build();


        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(52, 13.4))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(3.6)
                .build();

        Trajectory traj2cone1 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(4.05)
                .build();

        Trajectory traj2cone2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(4.65)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(4.5)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(51.5, -23.1, Math.toRadians(180)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(52, 13.4, Math.toRadians(270)))
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


        Trajectory traj6;

        if(location == Webcam.Location.ONE){
            traj6 = drive.trajectoryBuilder(traj3.end())
                    .lineToSplineHeading(new Pose2d(52 , 25, Math.toRadians(0)))
                    .build();
        }else if(location == Webcam.Location.TWO){
            traj6 = drive.trajectoryBuilder(traj3.end())
                    .lineToSplineHeading(new Pose2d(52 , 0, Math.toRadians(0)))
                    .build();
        } else {
            traj6 = drive.trajectoryBuilder(traj3.end())
                    .lineToSplineHeading(new Pose2d(52, -23, Math.toRadians(0)))
                    .build();
        }



        currentState = State.TRAJECTORY_0;
        drive.followTrajectoryAsync((traj0));



        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_0:
                    if(!drive.isBusy()){
                        drive.followTrajectoryAsync(traj1);
                        currentState = RightSideAuto.State.TRAJECTORY_1;
                        position = LinearSlidesArm.TurnValue.TOP.getTicks();
                        time.reset();
                    }
                    break;
                case TRAJECTORY_1:

                    if (!drive.isBusy()) {
                        currentState = RightSideAuto.State.TRAJECTORY_2;
                        if(conePlaced == 0){
                            drive.followTrajectoryAsync(traj2);
                        }else if (conePlaced == 1){
                            drive.followTrajectoryAsync(traj2cone1);
                        }else{
                            drive.followTrajectoryAsync(traj2cone2);
                        }
                    }

                        if(time.milliseconds()>2000){

                            position = LinearSlidesArm.TurnValue.TOP.getTicks();
                        }

                    break;
                case TRAJECTORY_2:

                    if (!drive.isBusy()) {



                        if(time.milliseconds()>1500){
                            currentState = RightSideAuto.State.TRAJECTORY_3;
                            drive.followTrajectoryAsync(traj3);
                            time.reset();
                        }
//                        else if (time.milliseconds()>1000){
//                            if(conePlaced == 0){
//                                position = 517;
//                            }else{
//                                position = 364;
//                            }
//                        }
                        else if (time.milliseconds()>400){
                            Jerry.intakeClaw.open();
                        }



                    }else{
                        time.reset();
                    }


                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {

                        if(conePlaced<2){
                            currentState = RightSideAuto.State.TRAJECTORY_4;
                            drive.followTrajectoryAsync(traj4);
                        }else{
                            currentState = RightSideAuto.State.TRAJECTORY_6;
                            drive.followTrajectoryAsync(traj6);
                        }


                    }

                    if (time.milliseconds()>300){
                        if(conePlaced == 0){
                            position = 517;
                        }else{
                            position = 364;
                        }
                    }

                    break;
                case TRAJECTORY_4:



                    if (!drive.isBusy()) {
                        currentState = RightSideAuto.State.GRAB_CONE;
                        time.reset();
                    }

                    break;
                case GRAB_CONE:





                    Jerry.intakeClaw.close();



                    if(time.milliseconds()>1500){
                        conePlaced++;
                        currentState = RightSideAuto.State.TRAJECTORY_1;


                        drive.followTrajectoryAsync(traj5);
                        time.reset();

                    }else if(time.milliseconds()>1000){
                        position = LinearSlidesArm.TurnValue.CONES.getTicks();
                    }
                    break;
                case TRAJECTORY_6:
                    if(!drive.isBusy()){
                        currentState = RightSideAuto.State.IDLE;
                    }

                    if(time.milliseconds()>1500){
                        position = 0;
                    }
                    break;
                case IDLE:

                    break;
            }

            drive.update();

            if(!Jerry.slides.isBusy()){
                Jerry.slides.stopArm();
            }

            Jerry.slides.moveToPosition(position, power);



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