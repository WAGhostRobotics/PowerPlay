package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.TeleOpParent;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Left Side", group = "competition")
public class LeftSideAuto extends LinearOpMode {
    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    LeftSideAuto.State currentState = LeftSideAuto.State.IDLE;

    double power = 1;
    int position = 0;

    enum State {
        GO_TO_PLACE,
        OUTTAKE_EXTEND,
        OUTTAKE_RETRACT,
        INTAKE_FULLY_EXTEND,
        INTAKE_GRAB,
        DONE_GRABBING,
        RETRACT_READY,
        SLIDES_RETRACT,
        PIVOT_RETRACT,
        OUTTAKE_READY,
        PARK,
        IDLE
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // feedback after OpMode started
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        int intakePosition = 0;
        int outtakePosition = 0;
        int armPosition = Arm.TurnValue.PARTIAL.getTicks();
        int cone = 1;

        State state;


        Tom.init(hardwareMap, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,0));


        Trajectory goToCone = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(51.5, 6, Math.toRadians(270)))
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


        }

        Trajectory park;

        if(location == Webcam.Location.ONE){
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineTo(new Vector2d(52, 13.6), Math.toRadians(0))
                    .build();
        }else if(location == Webcam.Location.TWO){
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineTo(new Vector2d(52, 13.6), Math.toRadians(0))
                    .build();
        }else{
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineTo(new Vector2d(52, 13.6), Math.toRadians(0))
                    .build();
        }

        state = State.GO_TO_PLACE;
        drive.followTrajectoryAsync(goToCone);


        while(opModeIsActive()){


            //INTAKE SLIDES UPDATE
            Tom.intake.moveToPosition(intakePosition, Tom.intake.getAdjustedPower());
            if(!Tom.intake.isBusy()){
                Tom.intake.stopArm();
            }


            //OUTTAKE SLIDES UPDATE
            Tom.outtake.moveToPosition(outtakePosition);
            if(!Tom.outtake.isBusy()){
                Tom.outtake.stopArm();
            }


            Tom.arm.moveToPosition(armPosition, Tom.arm.getAdjustedPower(armPosition));

            if(Tom.claw.isIn()){
                Tom.claw.in();
            }else{
                Tom.claw.out();
            }

            if(Tom.claw.isOpen()){
                Tom.claw.open();
            }else{
                Tom.claw.close();
            }

            drive.update();


            //OUTTAKE STATE MACHINE
            switch (state) {
                case GO_TO_PLACE:
                    if(!drive.isBusy()){
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        intakePosition = IntakeSlides.TurnValue.ALMOST_DONE.getTicks();

                        state = State.OUTTAKE_EXTEND;

                        switch(cone){
                            case 1:
                                armPosition = Arm.TurnValue.CONE1.getTicks();
                                break;
                            case 2:
                                armPosition = Arm.TurnValue.CONE2.getTicks();
                                break;
                            case 3:
                                armPosition = Arm.TurnValue.CONE3.getTicks();
                                break;
                            case 4:
                                armPosition = Arm.TurnValue.CONE4.getTicks();
                                break;
                            case 5:
                                armPosition = Arm.TurnValue.CONE5.getTicks();
                                break;
                            case 6:
                                state = State.PARK;
                                drive.followTrajectoryAsync(park);

                        }
                        cone++;
                        Tom.claw.out();

                    }
                    break;
                case OUTTAKE_EXTEND:
                    if(Tom.outtake.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
//                        armPosition = Arm.TurnValue.EXTENDED.getTicks();
//                        Tom.claw.out();
                        state = State.OUTTAKE_RETRACT;

                    }
                    break;
                case OUTTAKE_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.isFinished()){

                        intakePosition = IntakeSlides.TurnValue.ALMOST_DONE.getTicks();
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        state = State.INTAKE_FULLY_EXTEND;
                    }
                    break;
                case INTAKE_FULLY_EXTEND:
                    if(Tom.intake.isFinished()){
                        intakePosition = IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks();
                        state = State.INTAKE_GRAB;
                    }
                    break;
                case INTAKE_GRAB:
                    if(Tom.intake.isFinished()){
                        Tom.claw.close();
                        state = State.DONE_GRABBING;
                    }
                    break;
                case DONE_GRABBING:
                    if(Tom.claw.clawIsFinished()){
                        armPosition = Arm.TurnValue.LOW.getTicks();
                        state = State.RETRACT_READY;
                    }
                    break;
                case RETRACT_READY:
                    if(Tom.arm.isFinished()){
                        intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        armPosition = Arm.TurnValue.PARTIAL.getTicks();
                        Tom.claw.in();
                        state = State.SLIDES_RETRACT;
                    }
                    break;
                case SLIDES_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.isFinished()){
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        armPosition = Arm.TurnValue.RETRACTED.getTicks();
                        Tom.claw.in();


                        state = State.PIVOT_RETRACT;
                    }
                    break;
                case PIVOT_RETRACT:
                    if(Tom.intake.isFinished() &&Tom.arm.isFinished()){
                        Tom.claw.open();
                        armPosition = Arm.TurnValue.PARTIAL.getTicks();
                        state = State.OUTTAKE_READY;
                    }
                    break;
                case OUTTAKE_READY:
                    if(Tom.claw.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        intakePosition = IntakeSlides.TurnValue.PARTIAL.getTicks();
                        armPosition = Arm.TurnValue.EXTENDED.getTicks();
                        Tom.claw.out();
                        state = State.OUTTAKE_EXTEND;
                    }
                    break;

                case PARK:
                    if(!drive.isBusy()){
                        state = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;

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