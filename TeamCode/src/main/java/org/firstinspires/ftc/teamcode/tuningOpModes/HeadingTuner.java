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
@Autonomous(name = "HeadingTuner", group = "tuning")
public class HeadingTuner extends LinearOpMode {


    public static double p = MotionPlanner.headingControl.getP(), i = MotionPlanner.headingControl.getI(), d = MotionPlanner.headingControl.getD();





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


        double angle = 90;
        motionPlanner.startTrajectory(new Bezier(
                angle,
                new Point(0,0),
                new Point(0, 0)
        ));


        while (!isStopRequested()) {

            motionPlanner.headingControl.setPID(p, i, d);
            motionPlanner.update();

            if(motionPlanner.isFinished()){

                ElapsedTime wait = new ElapsedTime();

                while(wait.seconds()<3){

                }

                motionPlanner.startTrajectory(new Bezier(
                        angle += 90,
                        new Point(0,0),
                        new Point(0, 0)
                ));
            }


            telemetry.addData("Heading Error", motionPlanner.getHeadingError());
            telemetry.addData("Target", 0);




            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

    }
}
