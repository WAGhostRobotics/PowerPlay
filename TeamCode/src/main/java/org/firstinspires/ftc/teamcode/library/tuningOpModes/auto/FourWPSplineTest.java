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
@Autonomous(name = "4 WayPoint Spline Test", group = "tuning")
public class FourWPSplineTest extends LinearOpMode {


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
        motionPlanner.startTrajectory(new Bezier(
                new Point(0,0),
                new Point(45, 0),
                new Point(12, 25),
                new Point(45, 25)
        ));


        while (!isStopRequested()) {

            motionPlanner.translationalControl.setPID(p, i, d);
            motionPlanner.update();

            if(motionPlanner.isFinished()){

                if(stop){
                    wait.reset();
                    stop = false;
                }



                if(wait.seconds()>3) {
                    stop = true;


                    if (forward) {
                        forward = false;
                        motionPlanner.startTrajectory(new Bezier(
                                new Point(45, 25),
                                new Point(12, 25),
                                new Point(45, 0),
                                new Point(0, 0)
                        ));
                    } else {
                        forward = true;
                        motionPlanner.startTrajectory(new Bezier(
                                new Point(0, 0),
                                new Point(45, 0),
                                new Point(12, 25),
                                new Point(45, 25)
                        ));
                    }
                }
            }else{
                stop = true;
            }


            telemetry.addData("X Error", motionPlanner.getSpline().getEndPoint().getX() - localizer.getX());
            telemetry.addData("Y Error", motionPlanner.getSpline().getEndPoint().getY() - localizer.getY());
            telemetry.addData("Target", 0);
            telemetry.update();




            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

    }
}
