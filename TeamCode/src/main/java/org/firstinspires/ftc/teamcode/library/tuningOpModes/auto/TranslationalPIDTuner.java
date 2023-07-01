package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

@Config
@Autonomous(name = "Translational PID Tuner", group = "tuning")
public class TranslationalPIDTuner extends LinearOpMode {


    public static double p = MotionPlanner.translationalControl.getP(), i = MotionPlanner.translationalControl.getI(), d = MotionPlanner.translationalControl.getD();




    @Override
    public void runOpMode() throws InterruptedException {



        boolean stop = true;
        ElapsedTime wait = new ElapsedTime();



        Localizer localizer = new Localizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();


        boolean forward = true;
        double distance = 40;
        motionPlanner.startTrajectory(new Bezier(new Point(0, 0), new Point(distance, 0)));

        double timeout = 7;
        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested()) {
            motionPlanner.translationalControl.setPID(p, i, d);
            motionPlanner.update();

            if(motionPlanner.isFinished()||timer.seconds()>timeout){

                if(stop){
                    wait.reset();
                    stop = false;
                }



                if(wait.seconds()>3) {
                    stop = true;
                    timer.reset();
                    if (forward) {
                        forward = false;
                        motionPlanner.startTrajectory(new Bezier(new Point(distance, 0), new Point(0, 0)));
                    } else {
                        forward = true;
                        motionPlanner.startTrajectory(new Bezier(new Point(0, 0), new Point(distance, 0)));
                    }
                }
            }else{
                stop = true;
            }


            telemetry.addData("Perpendicular Error", motionPlanner.getPerpendicularError());
            telemetry.addData("Target", 0);
            telemetry.addData("LowerBound", -1);
            telemetry.addData("UpperBound", 1);
            telemetry.addData("Heading Error", motionPlanner.getHeadingError());
            telemetry.addData("X - error", motionPlanner.getSpline().getEndPoint().getX() - localizer.getX());
            telemetry.addData("Y - error", motionPlanner.getSpline().getEndPoint().getY() - localizer.getY());
            telemetry.addData("End", motionPlanner.getSpline().getEndPoint().getX() + " " + motionPlanner.getSpline().getEndPoint().getY());
            telemetry.addLine(motionPlanner.getTelemetry());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

    }
}
