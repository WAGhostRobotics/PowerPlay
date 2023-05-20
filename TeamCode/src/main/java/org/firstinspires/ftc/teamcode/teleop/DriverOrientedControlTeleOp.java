package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.teleopDrive.DriveStyle;

@TeleOp(name = "DriverOriented TeleOp", group = "competition")
public class DriverOrientedControlTeleOp extends TeleOpParent {

    @Override
    public void runOpMode() throws InterruptedException {
        super.type = DriveStyle.DriveType.DRIVERORIENTED;
        super.runOpMode();
    }
}