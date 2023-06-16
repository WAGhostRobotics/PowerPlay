package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

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
@Autonomous(name = "Heading End Tuner", group = "tuning")
public class HeadingEndTuner extends LinearOpMode {


    public static double p = MotionPlanner.headingControlEnd.getP(), i = MotionPlanner.headingControlEnd.getI(), d = MotionPlanner.headingControlEnd.getD();

    public static double trackWidth = Localizer.LATERAL_DISTANCE;




    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);

        boolean stop = true;
        ElapsedTime wait = new ElapsedTime();


        Localizer localizer = new Localizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

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

            motionPlanner.headingControlEnd.setPID(p, i, d);
            Localizer.LATERAL_DISTANCE = trackWidth;
            motionPlanner.update();

            if(motionPlanner.isFinished()){

                if(stop){
                    wait.reset();
                    stop = false;
                }



                if(wait.seconds()>3) {
                    stop = true;

                    motionPlanner.startTrajectory(new Bezier(
                            angle += 90,
                            new Point(0, 0),
                            new Point(0, 0)
                    ));
                }
            }else{
                stop = true;
            }


            telemetry.addData("Heading Error", motionPlanner.getHeadingError());
            telemetry.addData("Target", 0);
            telemetry.update();




            PhotonCore.CONTROL_HUB.clearBulkCache();
        }

    }
}
