/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;


@Config
@TeleOp(name = "Wonk TeleOp", group = "competition")
public class WonkControlTeleOp extends LinearOpMode {

    public double power = 0.8;



    @Override
    public void runOpMode() throws InterruptedException {



        Localizer localizer = new Localizer(hardwareMap);
        Drivetrain drive = new MecanumDrive(hardwareMap);



        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();

        waitForStart();

        WonkyDrive wonk = new WonkyDrive(this, hardwareMap, localizer, drive);

        while (opModeIsActive() && !isStopRequested()) {

            //DRIVETRAIN STUFF


//            // re-initializes imu to correct heading if teleop starts at the wrong heading
            if (gamepad2.left_stick_button){
                wonk.initIMU();
            }

            wonk.drive(gamepad2, power);
            PhotonCore.CONTROL_HUB.clearBulkCache();

            telemetry.addLine(wonk.getTelemetry());
            telemetry.update();

        }
    }
}
