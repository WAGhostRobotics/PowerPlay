package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.library.DriveStyle;
import org.firstinspires.ftc.teamcode.library.DriverOrientedControl;

@TeleOp(name = "Sketchy TeleOp", group = "competition")
public class SketchyTeleOp extends TeleOpParent {

    Orientation angles;
    Acceleration gravity;

    DriverOrientedControl drive;

    DriveStyle.DriveType type = DriveStyle.DriveType.MECANUMARCADE;


    public double power = 0.85;

    public int position = LinearSlidesArm.TurnValue.GROUND.getTicks();

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {






        GamepadEx driverOp = new GamepadEx(gamepad2);
        ToggleButtonReader aReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );


        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);


        Jerry.init(hardwareMap, false);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        drive.setPoseEstimate(new Pose2d());

        telemetry.addLine("Are you sure about this??");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {




            if(Jerry.slides.getTicks()>= LinearSlidesArm.TurnValue.BOTTOM.getTicks()+40){
                power = 0.2;
            }

            Jerry.slides.moveToPosition(position);

            if(!Jerry.slides.isBusy()){
                Jerry.slides.stopArm();
            }


            //re-initializes imu to correct heading if teleop starts at the wrong heading
            if(gamepad2.left_stick_button){
                drive.setPoseEstimate(new Pose2d());
            }



            if (aReader.getState()) {
                power = 0.2;
            } else {
                power = 0.85;
            }
            aReader.readValue();



//
            if(gamepad1.left_bumper||gamepad2.left_bumper){
                Jerry.intakeClaw.open();
            }

            if(gamepad1.right_bumper||gamepad2.right_bumper){
                Jerry.intakeClaw.close();
            }






            telemetry.addData("Arm position", Jerry.slides.getTicks());
            telemetry.update();



            if(gamepad1.a||gamepad2.a){
                position = LinearSlidesArm.TurnValue.GROUND.getTicks();
            }

            if(gamepad1.x||gamepad2.x){
                position = LinearSlidesArm.TurnValue.BOTTOM.getTicks();
            }

            if(gamepad1.y||gamepad2.y) {
                position = LinearSlidesArm.TurnValue.MID.getTicks();
            }

            if(gamepad1.b||gamepad2.b) {
                position = LinearSlidesArm.TurnValue.TOP.getTicks();
            }

            if(gamepad1.dpad_left||gamepad2.dpad_left){
                position = LinearSlidesArm.TurnValue.CONES.getTicks();
            }

            if((gamepad1.right_trigger >= 0.1 || gamepad2.right_trigger >= 0.1)&&position<=3250) {
                position += 25;
            }
            if((gamepad1.left_trigger >= 0.1 || gamepad2.left_trigger >= 0.1)) {
                position -= 25;
            }


            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad2.right_stick_x
                            )
                    );

                    if (gamepad2.dpad_up) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL
                        drive.setPoseEstimate(new Pose2d(0, 0, poseEstimate.getHeading()));

                        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                                .lineTo(new Vector2d(10, 0))
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad2.dpad_down) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        drive.setPoseEstimate(new Pose2d(0, 0, poseEstimate.getHeading()));

                        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                                .lineTo(new Vector2d(-10, 0))
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.dpad_right) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }



        }
    }
