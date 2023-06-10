/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;


@TeleOp(name = "Wonk TeleOp", group = "competition")
public class WonkControlTeleOp extends LinearOpMode {

    public double power = 0.8;



    @Override
    public void runOpMode() throws InterruptedException {


        Tom.init(hardwareMap, true);

        MecanumDrive drive = new MecanumDrive(
                Tom.frontLeft,
                Tom.frontRight,
                Tom.backLeft,
                Tom.backRight
        );


        Localizer localizer = new Localizer(hardwareMap);


        WonkyDrive wonk = new WonkyDrive(hardwareMap, localizer);

        waitForStart();

        while (opModeIsActive()) {

            //DRIVETRAIN STUFF


//            // re-initializes imu to correct heading if teleop starts at the wrong heading
//            if (gamepad2.left_stick_button){
//                Tom.initIMU();
//            }

            wonk.drive(gamepad2, 1);



            telemetry.addData("X", wonk.getX());
            telemetry.addData("Y", wonk.getY());
            telemetry.addData("Angle", wonk.getCurrentHeading());
            telemetry.addData("Ac", wonk.getAc());
            telemetry.addLine(wonk.getTelemetry());
            telemetry.update();

        }
    }
}
