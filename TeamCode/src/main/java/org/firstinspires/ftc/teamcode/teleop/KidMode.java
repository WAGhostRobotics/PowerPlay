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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CommandBase.AutoStackTeleOp;
import org.firstinspires.ftc.teamcode.CommandBase.Collect;
import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.Latch;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.drivetrain.DriveStyle;
import org.firstinspires.ftc.teamcode.library.teleopDrive.DriverOrientedControl;

@TeleOp(name = "KidMode", group = "competition")
public class KidMode extends LinearOpMode {

    Orientation angles;
    Acceleration gravity;

    DriverOrientedControl drive;

    DriveStyle.DriveType type = DriveStyle.DriveType.DRIVERORIENTED;


    public double power = 0.8;
    public double turningMultiplier = 0.8;


    enum IntakeState{
        SLIDES_RETRACT,
        CLAW_OPEN,
        PIVOT_RETRACT,
        IDLE
    }

    enum State {
        SLIDES_RETRACT,
        PLACE_CONE,
        CLAW_OPEN,
        PIVOT_RETRACT,
        OUTTAKE_READY,
        OUTTAKE_EXTEND,
        WAIT_FOR_OUTTAKE,
        OUTTAKE_RETRACT,
        IDLE
    }

    State autoPlaceState = State.IDLE;
    IntakeState intakeState = IntakeState.IDLE;
    double intakePower = 1;
    int intake = -1;

    Collect collectionScheduler = new Collect();
    AutoStackTeleOp autoStackScheduler = new AutoStackTeleOp();


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        GamepadEx driverOp = new GamepadEx(gamepad2);
        ToggleButtonReader rightStickReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        ToggleButtonReader aReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.A
        );

        ToggleButtonReader startReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.START
        );


        GamepadEx driverOp2 = new GamepadEx(gamepad1);

        ToggleButtonReader aReader2 = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.A
        );

        ToggleButtonReader startReader2 = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.START
        );






        Tom.init(hardwareMap, true);
        if(Tom.imu==null){
            Tom.initIMU();
        }

        waitForStart();

        Tom.intake.setTargetPosition(0);
        Tom.outtake.setTargetPosition(0);
        Tom.arm.setTargetPosition(Arm.TurnValue.PARTIAL.getPosition());
        Tom.claw.setClawPosition(Claw.OPEN);
        Tom.claw.setSpinPosition(Claw.IN);
        Tom.latch.setLatchPosition(Latch.OPEN);



        MecanumDrive drive = new MecanumDrive(
                Tom.frontLeft,
                Tom.frontRight,
                Tom.backLeft,
                Tom.backRight
        );

        collectionScheduler.stop();
        autoStackScheduler.stop();

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
                        turningMultiplier * power * (Math.pow(driverOp.getRightX(), 3)),
                        Tom.imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }

            if(Tom.intake.getTargetPosition()>=IntakeSlides.TurnValue.PARTIAL.getTicks()&& Tom.arm.getTargetPosition() <= Arm.TurnValue.LOW.getPosition()){
                turningMultiplier = 0.4;
            }else{
                turningMultiplier = 0.8;
            }


            //re-initializes imu to correct heading if teleop starts at the wrong heading
            if (gamepad2.left_stick_button){
                Tom.initIMU();
            }
            if (rightStickReader.getState()) {
                power = 0.2;
            } else {
                power = 0.4;
            }
            rightStickReader.readValue();
            aReader.readValue();
            aReader2.readValue();
            startReader.readValue();
            startReader2.readValue();




            //OUTTAKE SLIDES UPDATE
            Tom.outtake.update();
            Tom.intake.update();
            Tom.arm.update();

            //OUTTAKE SLIDES CONTROLLER
            if(gamepad1.x){
                Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.RETRACTED.getTicks());
                Tom.latch.setLatchPosition(Latch.OPEN);

                collectionScheduler.stop();
                autoStackScheduler.stop();
            }

            if(gamepad1.b){
                Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.MID.getTicks());
                Tom.latch.setLatchPosition(Latch.CLOSE);

                collectionScheduler.stop();
                autoStackScheduler.stop();
            }

            if(gamepad1.y) {
                Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.TOP.getTicks());
                Tom.latch.setLatchPosition(Latch.CLOSE);

                collectionScheduler.stop();
                autoStackScheduler.stop();
            }


//            boolean button1 = gamepad1.right_bumper;
//            boolean button2 = gamepad1.left_bumper;
//
//            if(button1&&button2){
//                Tom.claw.setSpinPosition(Claw.OUT+Claw.AUTO_OUT_DIFFERENCE);
//            }




            telemetry.addData("Collector Index", collectionScheduler.getIndex());
            telemetry.addData("Done", collectionScheduler.isFinished());
            telemetry.addData("Outtake current", Tom.outtake.getCurrent());
            telemetry.addData("Intake current", Tom.intake.getCurrent());
            telemetry.addData("Outtake position", Tom.outtake.getTicks());
            telemetry.addData("Target Outtake Pos", Tom.outtake.getTargetPosition());
            telemetry.addData("Intake position", Tom.intake.getTicks());
            telemetry.addData("Target Intake Pos", Tom.intake.getTargetPosition());
            telemetry.addData("Arm Position", Tom.arm.getPosition());
            telemetry.addData("Target Arm Pos", Tom.arm.encoderPositionTranslation());
            telemetry.addData("Intake State", intake);

            telemetry.update();


            autoStackScheduler.update();
            collectionScheduler.update();



            //INTAKE CONTROLLER
            if(gamepad1.dpad_right){
                collectionScheduler.init();
                autoStackScheduler.stop();
            }

            if(gamepad1.dpad_down){

                Tom.intake.setTargetPosition(IntakeSlides.TurnValue.PARTIAL.getTicks());
                Tom.arm.setTargetPosition(Arm.TurnValue.EXTENDED.getPosition());
                Tom.claw.setClawPosition(Claw.OPEN);
                Tom.claw.setSpinPosition(Claw.OUT);

                collectionScheduler.stop();
                autoStackScheduler.stop();

            }

            if(startReader.wasJustReleased()){
                if(Tom.latch.getLatchPosition() == Latch.OPEN){
                    Tom.latch.setLatchPosition(Latch.CLOSE);
                }else{
                    Tom.latch.setLatchPosition(Latch.OPEN);
                }

            }

            if(gamepad1.dpad_left) {
                Tom.intake.setTargetPosition(IntakeSlides.TurnValue.EXTENDED.getTicks());
                Tom.arm.setTargetPosition(Arm.TurnValue.SUPER_EXTENDED.getPosition());
                Tom.claw.setClawPosition(Claw.OPEN);
                Tom.claw.setSpinPosition(Claw.OUT);

                collectionScheduler.stop();
                autoStackScheduler.stop();

            }

            if(gamepad1.dpad_up){
                Tom.intake.setTargetPosition(IntakeSlides.TurnValue.RETRACTED.getTicks());
                Tom.arm.setTargetPosition(Arm.TurnValue.LOW.getPosition());
                Tom.claw.setSpinPosition(Claw.OUT);

                collectionScheduler.stop();
                autoStackScheduler.stop();
            }





            //AUTO PLACE CONTROLLER (MASTER BUTTON)
            if(gamepad1.right_stick_button){
                autoStackScheduler.init();
                collectionScheduler.stop();
            }

            //CLAW CODE
            if (aReader2.wasJustReleased()){
                if (Tom.claw.isOpen()){
                    Tom.claw.setClawPosition(Claw.CLOSE);
                } else {
                    Tom.claw.setClawPosition(Claw.OPEN);
                }
            }


            //OUTTAKE MINOR ADJUSTMENTS
            if (-gamepad1.right_stick_y>0.5) {
                Tom.outtake.setTargetPosition(Tom.outtake.getTargetPosition()+10);

                collectionScheduler.stop();
                autoStackScheduler.stop();

            } else if (-gamepad1.right_stick_y<-0.5) {
                Tom.outtake.setTargetPosition(Tom.outtake.getTargetPosition()-10);

                collectionScheduler.stop();
                autoStackScheduler.stop();
            }

            //INTAKE MINOR ADJUSTMENT
            if (gamepad1.right_stick_x<-0.5) {
                Tom.intake.setTargetPosition(Tom.intake.getTargetPosition()+10);

                collectionScheduler.stop();
                autoStackScheduler.stop();
            } else if (gamepad1.right_stick_x>0.5) {
                Tom.intake.setTargetPosition(Tom.intake.getTargetPosition()-10);

                collectionScheduler.stop();
                autoStackScheduler.stop();
            }


            //ARM MINOR ADJUSTMENT

            if(gamepad1.back){

                Tom.arm.setTargetPosition(Tom.arm.getTargetPosition()-0.01);

                if(Tom.arm.getTargetPosition()<=0){
                    Tom.arm.setTargetPosition(0);
                }

                collectionScheduler.stop();
                autoStackScheduler.stop();
            }else if(gamepad1.left_stick_button){
                Tom.arm.setTargetPosition(Tom.arm.getTargetPosition()+0.01);

                if(Tom.arm.getTargetPosition()>=1){
                    Tom.arm.setTargetPosition(1);
                }

                collectionScheduler.stop();
                autoStackScheduler.stop();

            }

        }
    }
}