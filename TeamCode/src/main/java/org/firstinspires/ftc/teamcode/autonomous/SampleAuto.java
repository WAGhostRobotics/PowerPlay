package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.GoToPosition;
import org.firstinspires.ftc.teamcode.CommandBase.IntakeMove;
import org.firstinspires.ftc.teamcode.CommandBase.Latch;
import org.firstinspires.ftc.teamcode.CommandBase.OuttakeMove;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.library.Command;
import org.firstinspires.ftc.teamcode.library.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.SequentialCommand;
import  org.firstinspires.ftc.teamcode.CommandBase.GoToPosition;
import org.firstinspires.ftc.teamcode.core.Tom;

import java.util.ArrayList;

@Autonomous(name = "Sample Auto", group = "competition")
public class SampleAuto extends LinearOpMode {


    Pose2d goToConePosition = new Pose2d(58.65, -1.80, Math.toRadians(286));

    Trajectory goToCone;


    SequentialCommand scheduler = new SequentialCommand(
            new ArrayList<Command>(){
                {
//                    add(new GoToPosition(drive, goToCone));
                    add(new IntakeMove(0, Arm.TurnValue.PARTIAL.getPosition(), Claw.IN));
                    add(new ParallelCommand(


                            new ArrayList<Command>(){
                                {

                                    add(new OuttakeMove(OuttakeSlides.TurnValue.TOP.getTicks()));
                                    add(new SequentialCommand(

                                            new ArrayList<Command>(){
                                                {
                                                   add(new Wait(200));
                                                   add(new Latch(true));
                                                }
                                            }

                                    ));
                                    add(new IntakeMove(IntakeSlides.TurnValue.ALMOST_DONE.getTicks(),
                                            Arm.TurnValue.CONE1.getPosition(),
                                            Claw.OUT));

                                }
                            }


                    ));
                    add(new Wait(3000));
                    add(new ParallelCommand(
                            new ArrayList<Command>(){
                                {
                                    add(new OuttakeMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()));
                                    add(new Latch(false));
                                }
                            }
                    ));
                }


            }
    );

    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));

         goToCone = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(35, -1, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(goToConePosition, Math.toRadians(354))
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Idk if this is gonna work");
            telemetry.update();
        }

        scheduler.init();

        while(opModeIsActive() && !isStopRequested()){
            scheduler.update();
            if(scheduler.isFinished()){
                telemetry.addLine("done");
                telemetry.update();
            }else{
                telemetry.addData("Index", scheduler.getIndex());
                telemetry.update();
            }
        }
    }

}
