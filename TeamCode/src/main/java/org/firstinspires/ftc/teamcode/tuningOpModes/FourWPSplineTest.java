package org.firstinspires.ftc.teamcode.tuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

@Config
@Autonomous(name = "4 WayPoint Spline Test", group = "tuning")
public class FourWPSplineTest extends LinearOpMode {


    public static double p = 0, i = 0, d = 0;




    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);




        Localizer localizer = new Localizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer);

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
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

                ElapsedTime wait = new ElapsedTime();

                while(wait.seconds()<3){

                }


                if(forward){
                    forward = false;
                    motionPlanner.startTrajectory(new Bezier(
                            new Point(45,25),
                            new Point(12, 25),
                            new Point(45, 0),
                            new Point(0, 20)
                    ));
                }else{
                    forward = true;
                    motionPlanner.startTrajectory(new Bezier(
                            new Point(0,0),
                            new Point(45, 0),
                            new Point(12, 25),
                            new Point(45, 25)
                    ));
                }
            }


            telemetry.addData("Perpendicular Error", motionPlanner.getPerpendicularError());
            telemetry.addData("Target", 0);




            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

    }
}
