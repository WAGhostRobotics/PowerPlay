package org.firstinspires.ftc.teamcode.library.tuningOpModes.swerve;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.SwerveDrive.ModuleV2;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;

@Config
@TeleOp(name = "Module Tuner", group = "competition")
public class ModuleTuner extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {




        ModuleV2 module = new ModuleV2(hardwareMap, "lf", "lfPivot", "lfEnc", true);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();

        double targetAngle = 0;

        waitForStart();

        while (opModeIsActive()) {

            //DRIVETRAIN STUFF

            if(gamepad1.a){
                targetAngle += 15;
            }

            if(gamepad1.b){
                targetAngle -= 15;
            }

            module.setTargetAngle(targetAngle);
            module.update();

            telemetry.addData("Target", 0);
            telemetry.addData("Angle Error", normalizeDegrees(module.getTargetAngle()- module.getModuleAngle()));
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();

        }
    }
}
