/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.DriveStyle;
import org.firstinspires.ftc.teamcode.library.WonkyDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name = "Wonk TeleOp", group = "competition")
public class WonkControlTeleOp extends LinearOpMode {

    public double power = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, true);
        if(Tom.imu==null){
            Tom.initIMU();
        }

        waitForStart();

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "lf");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rf");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "lr");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "rr");

//            frontLeft.setInverted(true);
//            frontRight.setInverted(true);
//            backLeft.setInverted(true);
//            backRight.setInverted(true);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Encoder xEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));
        Encoder yEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        WonkyDrive wonk = new WonkyDrive(imu, frontLeft, frontRight, backRight, backLeft, xEncoder, yEncoder);


        while (opModeIsActive()) {

            //DRIVETRAIN STUFF


//            // re-initializes imu to correct heading if teleop starts at the wrong heading
//            if (gamepad2.left_stick_button){
//                Tom.initIMU();
//            }

            wonk.drive(gamepad2, 0.6);

            telemetry.addData("X", wonk.getX());
            telemetry.addData("Y", wonk.getY());
            telemetry.addData("Angle", wonk.getAngle());
            telemetry.update();

        }
    }
}
