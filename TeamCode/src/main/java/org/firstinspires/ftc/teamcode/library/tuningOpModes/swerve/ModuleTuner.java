package org.firstinspires.ftc.teamcode.library.tuningOpModes.swerve;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive.ModuleV2;
import org.firstinspires.ftc.teamcode.util.AnalogEncoder;

@Config
@TeleOp(name = "Module Tuner", group = "competition")
public class ModuleTuner extends LinearOpMode {

    public static double targetAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");

        CRServo leftFrontPivot = hardwareMap.get(CRServo.class, "lfPivot");
        leftFrontPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder leftFrontEnc = new AnalogEncoder(hardwareMap.get(AnalogInput.class, "lfEnc"), 45, true);

        ModuleV2 module = new ModuleV2(leftFront, leftFrontPivot, leftFrontEnc);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();


        waitForStart();

        while (opModeIsActive()) {

            //DRIVETRAIN STUFF

            if(gamepad1.a){
                targetAngle += 15;
            }

            if(gamepad1.b){
                targetAngle -= 15;
            }

            if(gamepad1.y){
                module.setPower(1);
            }else if(gamepad1.x){
                module.setPower(-1);
            }else{
                module.setPower(0);
            }

//            if (gamepad1.dpad_right) {
//                module.setServoPower(ModuleV2.K_STATIC);
//            } else if (gamepad1.dpad_left) {
//                module.setServoPower(-ModuleV2.K_STATIC);
//            } else {
//                module.setServoPower(0);
//            }

            module.setTargetAngle(targetAngle);
            module.update();

            telemetry.addData("Target", module.getTargetAngle());
            telemetry.addData("Zero", module.getEncoder().getZero());
            telemetry.addData("Angle", module.getModuleAngle());
            telemetry.addData("Voltage", module.getEncoder().getVoltage());
            telemetry.addData("Radians Not Normalized", module.getEncoder().getCurrentPosition());
            telemetry.addData("Angle Error", module.getError());
            telemetry.addData("Power", module.getPower());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();

        }
    }
}
