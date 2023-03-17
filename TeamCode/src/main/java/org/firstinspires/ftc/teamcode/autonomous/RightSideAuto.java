package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.DetectionTest.FEET_PER_METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Collect;
import org.firstinspires.ftc.teamcode.CommandBase.CorrectionTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.IntakeMove;
import org.firstinspires.ftc.teamcode.CommandBase.OuttakeMove;
import org.firstinspires.ftc.teamcode.CommandBase.PlaceConeAuto;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.Latch;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.library.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.RunCommand;
import org.firstinspires.ftc.teamcode.library.SequentialCommand;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Right Side", group = "competition")
public class RightSideAuto extends LinearOpMode {


    Pose2d goToConePosition = new Pose2d(58.5, 0.75, Math.toRadians(73.37));

    Trajectory goToCone;
    Trajectory park;
    Trajectory correct;

    SampleMecanumDrive drive;

    Webcam.Location location;



    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));



        Trajectory goToCone = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(35, 0.25, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(goToConePosition, Math.toRadians(194.5))
                .build();
        while (!isStarted() && !isStopRequested()) {
            Tom.webcam.scanForTags();
            location = Tom.webcam.getLocation();
            AprilTagDetection tagOfInterest = Tom.webcam.getTagOfInterest();



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


        if (location == Webcam.Location.ONE) {
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineToConstantHeading(new Vector2d(58.569, 1), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(49.569, 26, 0), Math.toRadians(90))
                    .build();
        }else if (location == Webcam.Location.TWO) {
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineToConstantHeading(new Vector2d(54.569, 1), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(50.569, 0, 0), Math.toRadians(290))
                    .build();
        }else {
            park = drive.trajectoryBuilder(goToCone.end())
                    .splineToConstantHeading(new Vector2d(58.569, 1), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(50.569, -26, 0), Math.toRadians(270))
                    .build();
        }

        SequentialCommand scheduler = new SequentialCommand(
                new FollowTrajectory(drive, goToCone),
                new IntakeMove(0, Arm.TurnValue.PARTIAL.getPosition(), Claw.IN),
                new PlaceConeAuto(Arm.TurnValue.CONE1.getPosition(), IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks(), Claw.OUT+Claw.AUTO_OUT_DIFFERENCE),
                new PlaceConeAuto(Arm.TurnValue.CONE2.getPosition(), IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks(), Claw.OUT+Claw.AUTO_OUT_DIFFERENCE),
                new PlaceConeAuto(Arm.TurnValue.CONE3.getPosition(), IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks(), Claw.OUT+Claw.AUTO_OUT_DIFFERENCE),
                new PlaceConeAuto(Arm.TurnValue.CONE4.getPosition(), IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks(), Claw.OUT+Claw.AUTO_OUT_DIFFERENCE),
                new PlaceConeAuto(Arm.TurnValue.CONE5.getPosition(), IntakeSlides.TurnValue.AUTO_EXTENDED.getTicks(), Claw.OUT+Claw.AUTO_OUT_DIFFERENCE),
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.TOP.getTicks()),
                        new SequentialCommand(
                                new Wait(200),
                                new RunCommand(()-> Tom.latch.setLatchPosition(Latch.CLOSE)))),
                new Wait(300),
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                        new RunCommand(()->Tom.latch.setLatchPosition(Latch.OPEN))),
                new FollowTrajectory(drive, park)

        );

        SequentialCommand failsafePark = new SequentialCommand(
                new ParallelCommand(
                        new SequentialCommand(
                                new Wait(300),
                                new RunCommand(()->Tom.latch.setLatchPosition(Latch.CLOSE))
                        ),
                        new OuttakeMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                        new RunCommand(()->Tom.latch.setLatchPosition(Latch.OPEN)),
                        new FollowTrajectory(drive, park),
                        new SequentialCommand(
                                new IntakeMove(IntakeSlides.TurnValue.RETRACTED.getTicks(), Arm.TurnValue.PARTIAL.getPosition(), Claw.IN),
                        new Collect())
                )

        );

        SequentialCommand correction = new SequentialCommand(
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                        new IntakeMove(IntakeSlides.TurnValue.PLACE_CONE.getTicks(), Arm.TurnValue.PARTIAL.getPosition(), Claw.IN),
                        new CorrectionTrajectory(drive, goToConePosition))

        );

        scheduler.init();
        failsafePark.stop();
        correction.stop();


        while(opModeIsActive() && !isStopRequested()){
            if(autoTime.milliseconds()>27300&& !scheduler.isFinished()&&scheduler.getIndex() != scheduler.getSize()-1&& failsafePark.isFinished()){
                failsafePark.init();
                correction.stop();
                scheduler.stop();
            }else if(failsafePark.isFinished()){
                if(drive.inError(goToConePosition)&& correction.isFinished()&&scheduler.getIndex()>0&&scheduler.getIndex() != scheduler.getSize()-1&& !scheduler.isFinished()){
                    correction.init();
                }else if (correction.isFinished()){

                    scheduler.update();
                }
            }


//            telemetry.addData("Condition 1", drive.inError(goToConePosition));
//            telemetry.addData("Condition 2", correction.isFinished());
//            telemetry.addData("Condition 3", scheduler.getIndex()>0);
//            telemetry.addData("Condition 4", scheduler.getIndex() != scheduler.getSize()-1);
//            telemetry.addData("Condition 5", !scheduler.isFinished());
//            telemetry.update();


            failsafePark.update();
            correction.update();

            drive.update();
            Tom.outtake.update();
            Tom.intake.update();
            Tom.arm.update();
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
