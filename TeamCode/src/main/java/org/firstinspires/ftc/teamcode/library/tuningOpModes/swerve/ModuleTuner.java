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
        AnalogEncoder leftFrontEnc = new AnalogEncoder(hardwareMap.get(AnalogInput.class, "lfEnc"), -135, true);
        ModuleV2 module_lf = new ModuleV2(leftFront, leftFrontPivot, leftFrontEnc);

        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
        CRServo rightFrontPivot = hardwareMap.get(CRServo.class, "rfPivot");
        rightFrontPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder rightFrontEnc = new AnalogEncoder(hardwareMap.get(AnalogInput.class, "rfEnc"), 45, true);
        ModuleV2 module_rf = new ModuleV2(rightFront, rightFrontPivot, rightFrontEnc);

        DcMotor leftRear = hardwareMap.get(DcMotor.class, "lr");
        CRServo leftRearPivot = hardwareMap.get(CRServo.class, "lrPivot");
        leftRearPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder leftRearEnc = new AnalogEncoder(hardwareMap.get(AnalogInput.class, "lrEnc"), 45, true);
        ModuleV2 module_lr = new ModuleV2(leftRear, leftRearPivot, leftRearEnc);

        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rr");
        CRServo rightRearPivot = hardwareMap.get(CRServo.class, "rrPivot");
        rightRearPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder rightRearEnc = new AnalogEncoder(hardwareMap.get(AnalogInput.class, "rrEnc"), 45, true);
        ModuleV2 module_rr = new ModuleV2(rightRear, rightRearPivot, rightRearEnc);

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

//            if(gamepad1.y){
//                module_lf.setPower(1);
//                module_rf.setPower(1);
//                module_lr.setPower(1);
//                module_rr.setPower(1);
//            }else if(gamepad1.x){
//                module_lf.setPower(-1);
//                module_rf.setPower(-1);
//                module_lr.setPower(-1);
//                module_rr.setPower(-1);
//            }else{
//                module_lf.setPower(0);
//                module_rf.setPower(0);
//                module_lr.setPower(0);
//                module_rr.setPower(0);
//            }
//
//            if (gamepad1.dpad_right) {
//                module_lf.setServoPower(ModuleV2.K_STATIC);
//                module_rf.setServoPower(ModuleV2.K_STATIC);
//                module_lr.setServoPower(ModuleV2.K_STATIC);
//                module_rr.setServoPower(ModuleV2.K_STATIC);
//            } else if (gamepad1.dpad_left) {
//                module_lf.setServoPower(-ModuleV2.K_STATIC);
//                module_rf.setServoPower(-ModuleV2.K_STATIC);
//                module_lr.setServoPower(-ModuleV2.K_STATIC);
//                module_rr.setServoPower(-ModuleV2.K_STATIC);
//            } else {
//                module_lf.setServoPower(0);
//                module_rf.setServoPower(0);
//                module_lr.setServoPower(0);
//                module_rr.setServoPower(0);
//            }

            //module_lf.setTargetAngle(targetAngle);
          //  module_lf.update();

            telemetry.addData("Target", module_lf.getTargetAngle());
            telemetry.addData("Zero", module_lf.getEncoder().getZero());
            telemetry.addData("Angle", module_lf.getModuleAngle());
            telemetry.addData("Voltage", module_lf.getEncoder().getVoltage());
            telemetry.addData("Radians Not Normalized", module_lf.getEncoder().getCurrentPosition());
            telemetry.addData("Angle Error", module_lf.getError());
            telemetry.addData("Power", module_lf.getPower());
            telemetry.update();

            PhotonCore.CONTROL_HUB.clearBulkCache();

        }
    }
}
