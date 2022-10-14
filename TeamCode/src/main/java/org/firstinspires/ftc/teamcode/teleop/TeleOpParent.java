/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.component.LinearSlidesArm;
import org.firstinspires.ftc.teamcode.core.Jerry;
import org.firstinspires.ftc.teamcode.library.DriveStyle;
import org.firstinspires.ftc.teamcode.library.DriverOrientedControl;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

/**
 * feel free to change the name or group of your class to better fit your robot
 */
public class TeleOpParent extends LinearOpMode {

    /**
     * make sure to change these motors to your team's preference and configuration
     */


    Orientation angles;
    Acceleration gravity;

    DriverOrientedControl drive;

    DriveStyle.DriveType type = DriveStyle.DriveType.MECANUMARCADE;


    public double power = 1;


    @Override
    public void runOpMode() throws InterruptedException {


        GamepadEx driverOp = new GamepadEx(gamepad2);
        ToggleButtonReader aReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_BUMPER
        );

        GamepadEx driverOp2 = new GamepadEx(gamepad1);

        ToggleButtonReader xReader2 = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.RIGHT_BUMPER
        );


        waitForStart();

        Jerry.init(hardwareMap, true);
        if(Jerry.imu==null){
            Jerry.initIMU();
        }

        MecanumDrive drive = new MecanumDrive(
                Jerry.frontLeft,
                Jerry.frontRight,
                Jerry.backLeft,
                Jerry.backRight
        );


        while (opModeIsActive()) {


            if(type == DriveStyle.DriveType.MECANUMARCADE){
                drive.driveRobotCentric(
                        power*driverOp.getLeftX(),
                        power*driverOp.getLeftY(),
                        power*driverOp.getRightX(),
                        false
                );
            }else if(type == DriveStyle.DriveType.DRIVERORIENTED){
                drive.driveFieldCentric(
                        power*driverOp.getLeftX(),
                        power*driverOp.getLeftY(),
                        power*driverOp.getRightX(),
                        Jerry.imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }

            //re-initializes imu to correct heading if teleop starts at the wrong heading
            if(gamepad2.left_stick_button){
                Jerry.initIMU();
            }


            if (aReader.getState()) {
                power = 0.2;
            } else {
                power = 1;
            }
            aReader.readValue();


            xReader.readValue();
            xReader2.readValue();

            if(xReader.wasJustReleased()||xReader2.wasJustReleased()){
                if(Jerry.intakeClaw.isOpen()){
                    Jerry.intakeClaw.close();
                }else{
                    Jerry.intakeClaw.open();
                }
            }



//            telemetry.addData("Claw position", Jerry.intakeClaw.getPosition());
//            telemetry.update();

            //makes small increments or decrements to claw position
            if(gamepad1.right_trigger >= 0.1 || gamepad2.right_trigger >= 0.1) {
                Jerry.arm.moveUp();
            }
            if(gamepad1.left_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
                Jerry.arm.moveDown();
            }


            if(gamepad1.a||gamepad2.a){
                Jerry.arm.moveArm(LinearSlidesArm.TurnValue.GROUND);
            }

            if(gamepad1.x||gamepad2.x){
                Jerry.arm.moveArm(LinearSlidesArm.TurnValue.GROUND);
            }

            if(gamepad1.y||gamepad2.y){
                Jerry.arm.moveArm(LinearSlidesArm.TurnValue.GROUND);
            }

            if(gamepad1.b||gamepad2.b) {
                Jerry.arm.moveArm(LinearSlidesArm.TurnValue.GROUND);
            }


            if(!Jerry.arm.isBusy()){
                Jerry.arm.stopArm();
            }






                if (gamepad2.dpad_left) {
                Jerry.servoIntake.in();
            } else if (gamepad2.dpad_right) {
                Jerry.servoIntake.out();
            } else {
                Jerry.servoIntake.stop();
            }
        }
    }
}