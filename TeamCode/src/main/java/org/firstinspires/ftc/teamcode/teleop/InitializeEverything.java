package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Tom;

@TeleOp(name = "Initialize Everything", group = "competition")
public class InitializeEverything extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Tom.init(hardwareMap, false);
        if(Tom.imu==null){
            Tom.initIMU();
        }

        waitForStart();
    }
}
