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
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
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

    enum State {
        SLIDES_RETRACT,
        PIVOT_RETRACT,
        OUTTAKE_READY,
        OUTTAKE_EXTEND,
        OUTTAKE_RETRACT,
        IDLE
    }

    State autoPlaceState = State.IDLE;
    IntakeState intakeState = IntakeState.IDLE;
    double intakePower = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx driverOp = new GamepadEx(gamepad2);
        ToggleButtonReader rightStickReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        ToggleButtonReader aReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );

        GamepadEx driverOp2 = new GamepadEx(gamepad1);

        ToggleButtonReader aReader2 = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.A
        );

        int intakePosition = 0;
        int outtakePosition = 0;
        int armPosition = Arm.TurnValue.PARTIAL.getTicks();
        double clawPosition = Claw.OPEN;
        double spinPosition = Claw.IN;

        Tom.init(hardwareMap, true);
        if(Tom.imu==null){
            Tom.initIMU();
        }

        waitForStart();



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
            if (rightStickReader.getState()) {
                power = 0.2;
            } else {
                power = 0.85;
            }
            rightStickReader.readValue();
            aReader.readValue();
            aReader2.readValue();


            Tom.claw.setClawPosition(clawPosition);


            Tom.claw.setSpinPosition(spinPosition);


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


//            //ARM UPDATE
            Tom.arm.moveToPosition(armPosition, Tom.arm.getAdjustedPower(armPosition));
//            Tom.arm.updateTargetPos(armPosition);
//            if(Tom.arm.isFinished()){
//                Tom.arm.moveToPosition(armPosition, 0.14);
//            }


            //TELEMETRY
            telemetry.addData("Outtake position", Tom.outtake.getTicks());
            telemetry.addData("Target Outtake Pos", outtakePosition);
            telemetry.addData("Intake position", Tom.intake.getTicks());
            telemetry.addData("Target Intake Pos", intakePosition);
            telemetry.addData("Arm Position", Tom.arm.getTicks());
            telemetry.addData("Target Arm Pos", armPosition);
            telemetry.addData("Arm Power", Tom.arm.getAdjustedPower(armPosition));
            telemetry.update();




            //INTAKE SLIDES UPDATE
            Tom.intake.moveToPosition(intakePosition, Tom.intake.getAdjustedPower());
            if(!Tom.intake.isBusy()){
                Tom.intake.stopArm();
            }


            //INTAKE CONTROLLER
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                armPosition = Arm.TurnValue.PARTIAL.getTicks();
                spinPosition = Claw.IN;

                intakeState = IntakeState.SLIDES_RETRACT;
                autoPlaceState = State.IDLE;
            }

            if(gamepad1.dpad_down || gamepad2.dpad_down){
                intakePosition = IntakeSlides.TurnValue.PARTIAL.getTicks();
                armPosition = Arm.TurnValue.EXTENDED.getTicks();
                spinPosition = Claw.OUT;
                clawPosition = Claw.OPEN;
                intakeState = IntakeState.IDLE;
                autoPlaceState = State.IDLE;

            }

            if(gamepad1.dpad_left || gamepad2.dpad_left) {
                intakePosition = IntakeSlides.TurnValue.EXTENDED.getTicks();
                armPosition = Arm.TurnValue.EXTENDED.getTicks();
                spinPosition = Claw.OUT;
                clawPosition = Claw.OPEN;
                intakeState = IntakeState.IDLE;
                autoPlaceState = State.IDLE;

            }

            if(gamepad1.dpad_right || gamepad2.dpad_right){
                armPosition = Arm.TurnValue.LOW.getTicks();
                spinPosition = Claw.OUT;
            }


            //INTAKE STATE MACHINE
            switch (intakeState) {
                case SLIDES_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.spinIsFinished()){
                        intakePower = 0.2;
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        armPosition = Arm.TurnValue.RETRACTED.getTicks();

                        intakeState = IntakeState.PIVOT_RETRACT;
                    }
                    break;
                case PIVOT_RETRACT:
                    if(Tom.intake.isFinished() &&Tom.arm.isFinished()){
                        clawPosition = Claw.OPEN;
                        armPosition = Arm.TurnValue.PARTIAL.getTicks();
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
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.spinIsFinished()){
                        intakePower = 0.5;
                        intakePosition = IntakeSlides.TurnValue.PLACE_CONE.getTicks();
                        armPosition = Arm.TurnValue.RETRACTED.getTicks();

                        autoPlaceState = State.PIVOT_RETRACT;
                    }
                    break;
                case PIVOT_RETRACT:
                    if(Tom.intake.isFinished() &&Tom.arm.isFinished()){
                        clawPosition = Claw.OPEN;
                        armPosition = Arm.TurnValue.PARTIAL.getTicks();
                        intakePower = 1;
                        autoPlaceState = State.OUTTAKE_READY;
                    }
                    break;
                case OUTTAKE_READY:
                    if(Tom.claw.clawIsFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.TOP.getTicks();
                        intakePosition = IntakeSlides.TurnValue.PARTIAL.getTicks();
                        armPosition = Arm.TurnValue.EXTENDED.getTicks();
                        spinPosition = Claw.OUT;
                        autoPlaceState = State.OUTTAKE_EXTEND;
                    }
                    break;
                case OUTTAKE_EXTEND:
                    if(Tom.outtake.isFinished()){
                        outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
//                        intakePosition = IntakeSlides.TurnValue.PARTIAL.getTicks();
//                        armPosition = Arm.TurnValue.EXTENDED.getTicks();
//                        Tom.claw.out();
                        autoPlaceState = State.OUTTAKE_RETRACT;

                    }
                    break;
                case OUTTAKE_RETRACT:
                    if(Tom.intake.isFinished() && Tom.outtake.isFinished()&&Tom.arm.isFinished()&&Tom.claw.spinIsFinished()){
                        autoPlaceState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;

            }

            //AUTO PLACE CONTROLLER (MASTER BUTTON)
            if(gamepad1.right_stick_button){
                intakePosition = IntakeSlides.TurnValue.RETRACTED.getTicks();
                outtakePosition = OuttakeSlides.TurnValue.RETRACTED.getTicks();
                armPosition = Arm.TurnValue.PARTIAL.getTicks();
                spinPosition = Claw.IN;
                autoPlaceState = State.SLIDES_RETRACT;

                intakeState = IntakeState.IDLE;
            }

            //CLAW CODE
            if (aReader.wasJustReleased()|| aReader2.wasJustReleased()){
                if (Tom.claw.isOpen()){
                    clawPosition = Claw.CLOSE;
                } else {
                    clawPosition = Claw.OPEN;
                }
            }


            //OUTTAKE MINOR ADJUSTMENTS
            if (gamepad1.right_trigger >= 0.1 || gamepad2.right_trigger >= 0.1) {
                outtakePosition += 10;
            } else if (gamepad1.left_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
                outtakePosition -= 10;
            }

            //INTAKE MINOR ADJUSTMENT
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakePosition += 10;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakePosition -= 10;
            }


            //ARM MINOR ADJUSTMENT

            if(gamepad1.back){
                armPosition -= 40;

            }else if(gamepad1.left_stick_button){
                armPosition += 40;

            }

        }
    }
}