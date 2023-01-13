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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.DriveStyle;
import org.firstinspires.ftc.teamcode.library.DriverOrientedControl;


public class TeleOpParent extends LinearOpMode {

    Orientation angles;
    Acceleration gravity;

    DriverOrientedControl drive;

    DriveStyle.DriveType type = DriveStyle.DriveType.MECANUMARCADE;


    public double power = 1;


    enum IntakeState{
        SLIDES_RETRACT,
        PIVOT_RETRACT,
        IDLE
    }

    enum AutoPlaceState{
        SLIDES_RETRACT,
        PIVOT_RETRACT,
        OUTTAKE_READY,
        OUTTAKE_EXTEND,
        OUTTAKE_RETRACT,
        INTAKE_EXTEND,
        IDLE
    }

    AutoPlaceState autoPlaceState = AutoPlaceState.IDLE;
    IntakeState intakeState = IntakeState.IDLE;
    double intakePower = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx driverOp = new GamepadEx(gamepad2);
        ToggleButtonReader aReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        ToggleButtonReader xReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );

        GamepadEx driverOp2 = new GamepadEx(gamepad1);

        ToggleButtonReader xReader2 = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.A
        );

        int intakePosition = 0;
        int outtakePosition = 0;

        waitForStart();

        Tom.init(hardwareMap, true);
        if(Tom.imu==null){
            Tom.initIMU();
        }

        MecanumDrive drive = new MecanumDrive(
                Tom.frontLeft,
                Tom.frontRight,
                Tom.backLeft,
                Tom.backRight
        );


        while (opModeIsActive()) {

            //DRIVETRAIN STUFF
            if (type == DriveStyle.DriveType.MECANUMARCADE){
                drive.driveRobotCentric(
                        power * driverOp.getLeftX(),
                        power * driverOp.getLeftY(),
                        power * driverOp.getRightX(),
                        false
                );
            } else if (type == DriveStyle.DriveType.DRIVERORIENTED){
                drive.driveFieldCentric(
                        power * (Math.pow(driverOp.getLeftX(), 3)),
                        power * (Math.pow(driverOp.getLeftY(), 3)),
                        power * (Math.pow(driverOp.getRightX(), 3)),
                        Tom.imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }

            //re-initializes imu to correct heading if teleop starts at the wrong heading
            if (gamepad2.left_stick_button){
                Tom.initIMU();
            }
            if (aReader.getState()) {
                power = 0.2;
            } else {
                power = 0.85;
            }
            aReader.readValue();
            xReader.readValue();
            xReader2.readValue();


            //TELEMETRY
            telemetry.addData("Outtake position", Tom.outtake.getTicks());
            telemetry.addData("Intake position", Tom.intake.getTicks());
            telemetry.addData("Is finished", Tom.outtake.isFinished());
            telemetry.update();

            //OUTTAKE SLIDES UPDATE
            Tom.outtake.moveToPosition(outtakePosition);
            if(!Tom.outtake.isBusy()){
                Tom.outtake.stopArm();
            }

            //OUTTAKE SLIDES CONTROLLER
            if(gamepad1.x || gamepad2.x){
                outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
            }

            if(gamepad1.b || gamepad2.b){
                outtakePosition = OuttakeSlides.TurnValue.MID.getTicks();
            }

            if(gamepad1.y || gamepad2.y) {
                outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
            }




            //INTAKE SLIDES UPDATE
            Tom.intake.moveToPosition(intakePosition, power);
            if(!Tom.intake.isBusy()){
                Tom.intake.stopArm();
            }


            //INTAKE CONTROLLER
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                Tom.pivot.partial();
            }

            if(gamepad1.dpad_down || gamepad2.dpad_down){
                intakePosition = IntakeSlides.TurnValue.PARTIAL.getTicks();
                Tom.pivot.extend();
            }

            if(gamepad1.dpad_left || gamepad2.dpad_left) {
                intakePosition = IntakeSlides.TurnValue.EXTENDED.getTicks();
                Tom.pivot.extend();
            }

            if(gamepad1.dpad_right || gamepad2.dpad_right){
                Tom.pivot.low();
            }


            //INTAKE STATE MACHINE
            switch (intakeState) {
                case SLIDES_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.pivot.isFinished()){
                        intakePower = 0.5;
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        Tom.pivot.retract();

                        intakeState = IntakeState.PIVOT_RETRACT;
                    }
                    break;
                case PIVOT_RETRACT:
                    if(Tom.intake.isFinished() &&Tom.pivot.isFinished()){
                        Tom.claw.open();
                        Tom.pivot.partial();
                        intakePower = 1;
                        intakeState = IntakeState.IDLE;
                    }
                    break;
                case IDLE:
                    break;

            }


            //OUTTAKE STATE MACHINE
            switch (autoPlaceState) {
                case SLIDES_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.pivot.isFinished()){
                        intakePower = 0.5;
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        Tom.pivot.retract();

                        autoPlaceState = AutoPlaceState.PIVOT_RETRACT;
                    }
                    break;
                case PIVOT_RETRACT:
                    if(Tom.intake.isFinished() &&Tom.pivot.isFinished()){
                        Tom.claw.open();
                        Tom.pivot.partial();
                        intakePower = 1;
                        autoPlaceState = AutoPlaceState.OUTTAKE_READY;
                    }
                    break;
                case OUTTAKE_READY:
                    if(Tom.pivot.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        autoPlaceState = AutoPlaceState.OUTTAKE_EXTEND;
                    }
                    break;
                case OUTTAKE_EXTEND:
                    if(Tom.outtake.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                        intakePosition = IntakeSlides.TurnValue.PARTIAL.getTicks();
                        Tom.pivot.extend();
                        autoPlaceState = AutoPlaceState.OUTTAKE_RETRACT;

                    }
                    break;
                case OUTTAKE_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.pivot.isFinished()){
                        autoPlaceState = AutoPlaceState.IDLE;
                    }
                    break;
                case IDLE:
                    break;

            }

            //AUTO PLACE CONTROLLER (MASTER BUTTON)
            if(gamepad1.right_stick_button){
                autoPlaceState = AutoPlaceState.SLIDES_RETRACT;
            }


            //CLAW CODE
            if (xReader.wasJustReleased()|| xReader2.wasJustReleased()){
                if (Tom.claw.isOpen()){
                    Tom.claw.close();
                } else {
                    Tom.claw.open();
                }
            }


            //OUTTAKE MINOR ADJUSTMENTS
            if (gamepad1.right_trigger >= 0.1 || gamepad2.right_trigger >= 0.1) {
                outtakePosition += 40;
            } else if (gamepad1.left_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
                outtakePosition -= 40;
            }

            //INTAKE MINOR ADJUSTMENT
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakePosition += 40;
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakePosition -= 40;
            }
        }
    }
}