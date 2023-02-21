package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.TeleOpParent;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Park", group = "competition")
public class Park extends LinearOpMode {
    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    LeftSideAuto.State currentState = LeftSideAuto.State.IDLE;

    double power = 1;
    int position = 0;

    enum State {
        GO_TO_PLACE,
        WAIT_FOR_OUTTAKE,
        OUTTAKE_EXTEND,
        INTAKE_FULLY_EXTEND,
        INTAKE_GRAB,
        DONE_GRABBING,
        RETRACT_READY,
        SLIDES_RETRACT,
        WAIT_FOR_CLAW,
        CLAW_OPEN,
        PIVOT_RETRACT,
        OUTTAKE_READY,
        READY_TO_PARK,
        PARK,
        PARK2,
        BACK,
        IDLE
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // feedback after OpMode started
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        int intakePosition = 0;
        int outtakePosition = 0;
        double armPosition = Arm.TurnValue.PARTIAL.getPosition();
        double clawPosition = Claw.OPEN;
        double spinPosition = Claw.IN;
        int cone = 1;

        State state;


        Tom.init(hardwareMap, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,0));




        Trajectory readyToPark = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(49.569, -1.25, Math.toRadians(0)))
                .build();







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

            Tom.arm.moveToPosition(Arm.TurnValue.START_AUTO.getPosition());
            Tom.claw.setClawPosition(Claw.OPEN);

        }

        ElapsedTime autoTime = new ElapsedTime();
        autoTime.reset();

        Trajectory park;

        if (location == Webcam.Location.ONE) {
            park = drive.trajectoryBuilder(readyToPark.end())
                    .lineTo(new Vector2d(49.569, 25))
                    .build();
        }else if (location == Webcam.Location.TWO) {
            park = drive.trajectoryBuilder(readyToPark.end())
                    .lineTo(new Vector2d(49.569, 0))
                    .build();
        }else {
            park = drive.trajectoryBuilder(readyToPark.end())
                    .lineTo(new Vector2d(49.569, -24))
                    .build();
        }

        Trajectory back = drive.trajectoryBuilder(park.end())
                .back(20)
                .build();

        state = State.PARK;


        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(opModeIsActive() && !isStopRequested()){


            //INTAKE SLIDES UPDATE
            Tom.intake.moveToPosition(intakePosition, 0.9);
            if(!Tom.intake.isBusy()){
                Tom.intake.stopArm();
            }


            //OUTTAKE SLIDES UPDATE
            Tom.outtake.moveToPosition(outtakePosition, Tom.outtake.getAdjustedPower());
            if(!Tom.outtake.isBusy()){
                Tom.outtake.stopArm();
            }


            Tom.arm.moveToPosition(armPosition);

            Tom.claw.setClawPosition(clawPosition);


            Tom.claw.setSpinPosition(spinPosition);

            drive.update();


            //OUTTAKE STATE MACHINE
            switch (state) {
                case READY_TO_PARK:
                    if(time.milliseconds()>200){
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        state = State.PARK;
                    }
                    break;
                case PARK:
                    if(Tom.outtake.isFinished()){
                        drive.followTrajectoryAsync(readyToPark);
                        state = State.PARK2;
                    }
                    break;
                case PARK2:
                    if(!drive.isBusy()){
                        drive.followTrajectoryAsync(park);
                        state = State.IDLE;
                    }
                    break;
                case BACK:
                    if(!drive.isBusy()){
                        drive.followTrajectoryAsync(back);
                        state = State.IDLE;
                    }
                case IDLE:
                    break;

            }

            if(autoTime.milliseconds()>26500 && state != State.READY_TO_PARK && state != State.PARK && state != State.PARK2 && state != State.BACK && state != State.IDLE){
                armPosition = Arm.TurnValue.PARTIAL.getPosition();
                spinPosition = Claw.IN;
                intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                outtakePosition = Tom.outtake.getTicks();
                state = State.READY_TO_PARK;
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