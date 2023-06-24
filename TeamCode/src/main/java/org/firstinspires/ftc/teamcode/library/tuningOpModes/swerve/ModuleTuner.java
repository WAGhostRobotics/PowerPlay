package org.firstinspires.ftc.teamcode.library.tuningOpModes.swerve;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive.ModuleV2;

@Config
@TeleOp(name = "Module Tuner", group = "competition")
public class ModuleTuner extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {




        ModuleV2 module = new ModuleV2(hardwareMap, "lf", "lfPivot", "lfEnc", 0);


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
