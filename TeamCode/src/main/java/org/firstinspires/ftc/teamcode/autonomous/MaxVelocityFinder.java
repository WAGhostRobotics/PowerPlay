package org.firstinspires.ftc.teamcode.autonomous;



import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;


@Autonomous(name = "Max Velocity Finder", group = "competition")
public class MaxVelocityFinder extends LinearOpMode {

    Localizer localizer;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);




        localizer = new Localizer(hardwareMap);
        drive = new MecanumDrive(hardwareMap);

        double maxVelocity = 0;
        double velocity;
        double lastVelocity = 0;

        double accel;
        double maxAccel = 0;

        double lastx = 0;
        double lasty = 0;
        double currentX;
        double currentY;
        double lastTime = 0;
        double t;

        PhotonCore.enable();

        waitForStart();

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (!isStopRequested()) {

            if(time.seconds()<1){
                drive.driveMax(1, 0, 0, 0.7);

                currentX = getX();
                currentY = getY();

                t = time.seconds() - lastTime;

                velocity = Math.sqrt(
                        Math.pow(((currentX-lastx)/t), 2) + Math.pow(((currentY-lasty)/t), 2)
                );

                accel = Math.abs(velocity - lastVelocity)/t;

                lastVelocity = velocity;


                lastx = currentX;
                lasty = currentY;
                lastTime = time.seconds();

                maxVelocity = Math.max(maxVelocity, velocity);
                maxAccel = Math.max(maxAccel, accel);
                telemetry.addLine("Velocity" + maxVelocity);
                telemetry.addLine("Accel" + maxAccel);
                telemetry.update();
            }
//            else{
//
//                drive.driveMax(0, 0, 0, 0.7);
//
//                currentX = getX();
//                currentY = getY();
//
//                t = time.seconds() - lastTime;
//
//                velocity = Math.sqrt(
//                        Math.pow(((currentX-lastx)/t), 2) + Math.pow(((currentY-lasty)/t), 2)
//                );
//
//                accel = Math.abs(velocity - lastVelocity)/t;
//
//                lastVelocity = velocity;
//
//
//                lastx = currentX;
//                lasty = currentY;
//                lastTime = time.seconds();
//
//                maxVelocity = Math.max(maxVelocity, velocity);
//                maxAccel = Math.min(maxAccel, accel);
//                telemetry.addLine("Velocity" + maxVelocity);
//                telemetry.addLine("Accel" + maxAccel);
//                telemetry.update();
//            }

        }
    }

    public double getX(){
        return localizer.getRawY();
    }

    public double getY(){
        return localizer.getRawX();
    }
}

