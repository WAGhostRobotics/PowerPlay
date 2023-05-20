package org.firstinspires.ftc.teamcode.autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.Drive;
import org.firstinspires.ftc.teamcode.library.Localizer;
import org.firstinspires.ftc.teamcode.library.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.math.Bezier;
import org.firstinspires.ftc.teamcode.library.math.Point;


@Autonomous(name = "Max Velocity Finder", group = "competition")
public class MaxVelocityFinder extends LinearOpMode {

    Localizer localizer;
    Drive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        Tom.init(hardwareMap, false);




        localizer = new Localizer(hardwareMap);
        drive = new Drive(hardwareMap);

        double maxVelocity = 0;
        double velocity;
        double lastx = 0;
        double lasty = 0;
        double currentX;
        double currentY;
        double lastTime = 0;
        double t;

        waitForStart();

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (!isStopRequested()) {

            if(time.seconds()<6){
                drive.driveMax(1, 90, 0, 1);

                currentX = getX();
                currentY = getY();

                t = time.seconds() - lastTime;

                velocity = Math.sqrt(
                        Math.pow(((currentX-lastx)/t), 2) + Math.pow(((currentY-lasty)/t), 2)
                );


                lastx = currentX;
                lasty = currentY;
                lastTime = time.seconds();

                maxVelocity = Math.max(maxVelocity, velocity);
                telemetry.addLine("Velocity" + maxVelocity);
                telemetry.update();
            }

        }
    }

    public double getX(){
        return localizer.getRawY();
    }

    public double getY(){
        return localizer.getRawX();
    }
}

