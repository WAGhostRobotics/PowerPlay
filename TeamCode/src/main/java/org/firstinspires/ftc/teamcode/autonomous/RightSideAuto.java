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

@Autonomous(name = "Right Side", group = "competition")
public class RightSideAuto extends LinearOpMode {
    Webcam.Location location = null;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    LeftSideAuto.State currentState = LeftSideAuto.State.IDLE;

    double power = 1;
    int position = 0;

    enum State {
        GO_TO_PLACE,
        OUTTAKE_EXTEND,
        INTAKE_FULLY_EXTEND,
        INTAKE_GRAB,
        DONE_GRABBING,
        RETRACT_READY,
        SLIDES_RETRACT,
        CLAW_OPEN,
        PIVOT_RETRACT,
        OUTTAKE_READY,
        READY_TO_PARK,
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
        double armPosition = Arm.TurnValue.PARTIAL.getPosition();
        double clawPosition = Claw.OPEN;
        double spinPosition = Claw.IN;
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

            Tom.arm.moveToPosition(Arm.TurnValue.PARTIAL.getPosition());

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


        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(opModeIsActive() && !isStopRequested()){


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


            Tom.arm.moveToPosition(armPosition);

            Tom.claw.setClawPosition(clawPosition);


            Tom.claw.setSpinPosition(spinPosition);

            drive.update();


            //OUTTAKE STATE MACHINE
            switch (state) {
                case GO_TO_PLACE:
                    if(!drive.isBusy()){
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        intakePosition = IntakeSlides.TurnValue.ALMOST_DONE.getTicks();



                        armPosition = Arm.TurnValue.CONE1.getPosition();

                        cone++;
                        spinPosition = Claw.OUT;

                        state = State.OUTTAKE_EXTEND;


                    }
                    break;
                case OUTTAKE_EXTEND:
                    if(Tom.outtake.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
//                        armPosition = Arm.TurnValue.EXTENDED.getTicks();
//                        spinPosition = Claw.OUT;
                        state = State.INTAKE_FULLY_EXTEND;

                    }
                    break;
                case INTAKE_FULLY_EXTEND:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.spinIsFinished()){
                        intakePosition = IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks();
                        state = State.INTAKE_GRAB;
                    }
                    break;
                case INTAKE_GRAB:
                    if(Tom.intake.isFinished()){
                        spinPosition = Claw.CLOSE;
                        time.reset();
                        state = State.DONE_GRABBING;
                    }
                    break;
                case DONE_GRABBING:
                    if(time.milliseconds()>300){
                        armPosition = Arm.TurnValue.LOW.getPosition();
                        state = State.RETRACT_READY;
                    }
                    break;
                case RETRACT_READY:
                    if(Tom.arm.isFinished()){
                        intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        armPosition = Arm.TurnValue.PARTIAL.getPosition();
                        spinPosition = Claw.IN;
                        state = State.SLIDES_RETRACT;
                    }
                    break;
                case SLIDES_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.spinIsFinished()){
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        armPosition = Arm.TurnValue.RETRACTED.getPosition();


                        state = State.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if(Tom.intake.isFinished() &&Tom.arm.isFinished()){
                        clawPosition = Claw.OPEN;
                        time.reset();
                        state = State.PIVOT_RETRACT;
                    }
                    break;
                case PIVOT_RETRACT:
                    if(time.milliseconds()>300){
                        armPosition = Arm.TurnValue.PARTIAL.getPosition();
                        state = State.OUTTAKE_READY;
                    }
                    break;
                case OUTTAKE_READY:
                    if(Tom.arm.isFinished()){
                        state = State.OUTTAKE_EXTEND;
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        intakePosition = IntakeSlides.TurnValue.ALMOST_DONE.getTicks();
                        spinPosition = Claw.OUT;
                        switch(cone){
                            case 1:
                                armPosition = Arm.TurnValue.CONE1.getPosition();
                                break;
                            case 2:
                                armPosition = Arm.TurnValue.CONE2.getPosition();
                                break;
                            case 3:
                                armPosition = Arm.TurnValue.CONE3.getPosition();
                                break;
                            case 4:
                                armPosition = Arm.TurnValue.CONE4.getPosition();
                                break;
                            case 5:
                                armPosition = Arm.TurnValue.CONE5.getPosition();
                                break;
                            case 6:
                                state = State.READY_TO_PARK;
                                armPosition = Arm.TurnValue.PARTIAL.getPosition();
                                spinPosition = Claw.IN;
                                intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                                break;

                        }
                        cone++;


                    }
                    break;
                case READY_TO_PARK:
                    if(Tom.outtake.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        drive.followTrajectoryAsync(park);
                        state = State.PARK;
                    }
                    break;
                case PARK:
                    if(!drive.isBusy()&&Tom.outtake.isFinished()){
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