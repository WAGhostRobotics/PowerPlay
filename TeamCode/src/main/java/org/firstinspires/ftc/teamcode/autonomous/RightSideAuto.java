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
import org.firstinspires.ftc.teamcode.component.Latch;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.TeleOpParent;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
        GRAB_LAST,
        CORRECT_POSITION,
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
        double latchPosition = Latch.CLOSE;
        int cone = 1;
        boolean placing = false;

        State state;

        State lastState;


        Tom.init(hardwareMap, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,0));


//        Trajectory goToCone = drive.trajectoryBuilder(new Pose2d())
//                .lineToSplineHeading(new Pose2d(57.875
//                        , -1.75, Math.toRadians(74.5)))
//                .build();

        Pose2d goToConePosition = new Pose2d(57.875, -1.75, Math.toRadians(74.5));

        Trajectory goToCone = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(35, -0.25, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(goToConePosition, Math.toRadians(194.5))
                .build();

        Trajectory correctPosition;








        // updates feedback after initialization finished
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Tom.latch.setLatchPosition(Latch.OPEN);

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

            if(gamepad1.a){
                Tom.latch.setLatchPosition(Latch.CLOSE);
            }

        }

        ElapsedTime autoTime = new ElapsedTime();
        autoTime.reset();


        Trajectory park;

        if (location == Webcam.Location.ONE) {
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineToConstantHeading(new Vector2d(54.569, 1), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(50.569, 26, 0), Math.toRadians(60))
                    .build();
        }else if (location == Webcam.Location.TWO) {
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineToConstantHeading(new Vector2d(54.569, 1), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(50.569, 0, 0), Math.toRadians(290))
                    .build();
        }else {
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineToConstantHeading(new Vector2d(54.569, 1), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(50.569, -26, 0), Math.toRadians(290))
                    .build();
        }





        state = State.GO_TO_PLACE;
        lastState = State.GO_TO_PLACE;
        drive.followTrajectoryAsync(goToCone);


        ElapsedTime time = new ElapsedTime();
        time.reset();

        ElapsedTime stallingTime = new ElapsedTime();

        while(opModeIsActive() && !isStopRequested()){

            Pose2d error = drive.getPoseEstimate().minus(goToConePosition);
            telemetry.addData("X", error.getX());
            telemetry.addData("Y", error.getY());
            telemetry.addData("Heading", error.getHeading());
            telemetry.addData("Stalling", Tom.outtake.isStalling());
            telemetry.addData("State", state.name());
            telemetry.addData("Last State", lastState.name());
            telemetry.addData("Outtake current", Tom.outtake.getCurrent());
            telemetry.update();

            if(!Tom.outtake.isStalling()){
                stallingTime.reset();
            }

            if(state != State.CORRECT_POSITION){
                //INTAKE SLIDES UPDATE
                Tom.intake.moveToPosition(intakePosition, 1);
                if(!Tom.intake.isBusy()){
                    Tom.intake.stopArm();
                }


                //OUTTAKE SLIDES UPDATE
                Tom.outtake.moveToPosition(outtakePosition, 1);
                if(!Tom.outtake.isBusy()){
                    Tom.outtake.stopArm();
                }



                Tom.arm.moveToPosition(armPosition);

                Tom.claw.setClawPosition(clawPosition);


                Tom.claw.setSpinPosition(spinPosition);

                Tom.latch.setLatchPosition(latchPosition);
            }else{
                Tom.intake.moveToPosition(IntakeSlides.TurnValue.PARTIAL.getTicks(), 1);
                if(!Tom.intake.isBusy()){
                    Tom.intake.stopArm();
                }


                Tom.outtake.moveToPosition(OuttakeSlides.TurnValue.RETRACTED.getTicks(), 1);
                if(!Tom.outtake.isBusy()){
                    Tom.outtake.stopArm();
                }



                Tom.arm.moveToPosition(Arm.TurnValue.LOW.getPosition());

            }

            drive.update();

            if(state != State.CORRECT_POSITION && placing && drive.inError(goToConePosition)){
                lastState = state;
                state = State.CORRECT_POSITION;

                correctPosition = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(goToConePosition)
                        .build();

                drive.followTrajectoryAsync(correctPosition);
            }


            //OUTTAKE STATE MACHINE
            switch (state) {
                case CORRECT_POSITION:
                    if(!drive.isBusy()){
                        state = lastState;
                    }
                    break;
                case GO_TO_PLACE:
                    if(!drive.isBusy()){
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        intakePosition = IntakeSlides.TurnValue.ALMOST_DONE.getTicks();

                        armPosition = Arm.TurnValue.CONE1.getPosition();

                        cone++;
                        spinPosition = Claw.OUT;

                        placing = true;

                        state = State.WAIT_FOR_OUTTAKE;


                    }
                    break;
                case WAIT_FOR_OUTTAKE:
                    if(Tom.outtake.isFinished()){
                        time.reset();
//                        armPosition = Arm.TurnValue.EXTENDED.getTicks();
//                        spinPosition = Claw.OUT;
                        state = State.OUTTAKE_EXTEND;

                        if(cone == 7){
                            state = State.READY_TO_PARK;
                        }

                    }
                    break;
                case OUTTAKE_EXTEND:
                    if(time.milliseconds()>100){
                        latchPosition = Latch.OPEN;
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        time.reset();
//                        armPosition = Arm.TurnValue.EXTENDED.getTicks();
//                        spinPosition = Claw.OUT;
                        state = State.INTAKE_FULLY_EXTEND;

                        stallingTime.reset();

                    }
                    break;
                case INTAKE_FULLY_EXTEND:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.spinIsFinished()){
                        intakePosition = IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks();
                        state = State.INTAKE_GRAB;
                    }else if(stallingTime.milliseconds()>1700){
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        intakePosition = IntakeSlides.TurnValue.ALMOST_DONE.getTicks();
                        state = State.WAIT_FOR_OUTTAKE;
                    }
                    break;
                case INTAKE_GRAB:
                    if(Tom.intake.isFinished()){
                        clawPosition = Claw.CLOSE;
                        time.reset();
                        state = State.DONE_GRABBING;
                    }
                    break;
                case DONE_GRABBING:
                    if(time.milliseconds() > 200){
                        armPosition = Arm.TurnValue.LOW.getPosition();
                        state = State.RETRACT_READY;
                    }
                    break;
                case RETRACT_READY:
                    if(Tom.arm.isFinished()){
                        intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                        outtakePosition = OuttakeSlides.TurnValue.SUPER_RETRACTED.getTicks();
                        armPosition = Arm.TurnValue.PARTIAL.getPosition();
                        spinPosition = Claw.IN;
                        state = State.WAIT_FOR_CLAW;
                    }
                    break;
                case WAIT_FOR_CLAW:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.spinIsFinished()){
                        time.reset();


                        state = State.SLIDES_RETRACT;
                    }
                    break;
                case SLIDES_RETRACT:
                    if(time.milliseconds()>150){
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        armPosition = Arm.TurnValue.RETRACTED.getPosition();


                        state = State.CLAW_OPEN;
                    }
                    break;
                case CLAW_OPEN:
                    if(Tom.intake.isFinished() &&Tom.arm.isFinished()){
                        clawPosition = Claw.OPEN;
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        time.reset();
                        state = State.PIVOT_RETRACT;
                    }
                    break;

                case PIVOT_RETRACT:
                    if(time.milliseconds()>100){
                        latchPosition = Latch.CLOSE;
                        armPosition = Arm.TurnValue.PARTIAL.getPosition();
                        state = State.OUTTAKE_READY;
                    }
                    break;
                case OUTTAKE_READY:
                    if(Tom.arm.isFinished()){
                        state = State.WAIT_FOR_OUTTAKE;
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
                                armPosition = Arm.TurnValue.PARTIAL.getPosition();
                                spinPosition = Claw.IN;
                                intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                                break;

                        }
                        cone++;

                        time.reset();

                    }
                    break;
                case READY_TO_PARK:


                    if(time.milliseconds()>100){
                        placing = false;
                        latchPosition = Latch. OPEN;
                        outtakePosition = OuttakeSlides.TurnValue.SUPER_RETRACTED.getTicks();
                        state = State.PARK;
                    }
                    break;

                case PARK:
                    if(Tom.outtake.isFinished()||Tom.outtake.isStalling()){
                        drive.followTrajectoryAsync(park);
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        armPosition = Arm.TurnValue.RETRACTED.getPosition();
                        state = State.IDLE;
                    }
                    break;
                case GRAB_LAST:
                    if(Tom.intake.isFinished()&&Tom.arm.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                }
                break;
                case IDLE:
                    break;

            }

            if(autoTime.milliseconds()>27000 && state != State.READY_TO_PARK && state != State.PARK && state != State.IDLE && state != State.GRAB_LAST){
                armPosition = Arm.TurnValue.PARTIAL.getPosition();
                spinPosition = Claw.IN;
                intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                outtakePosition = Tom.outtake.getTicks();
                time.reset();
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