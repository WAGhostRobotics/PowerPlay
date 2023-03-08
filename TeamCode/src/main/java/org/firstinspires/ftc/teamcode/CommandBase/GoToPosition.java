package org.firstinspires.ftc.teamcode.CommandBase;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.library.Command;

public class GoToPosition extends Command {

    SampleMecanumDrive drive;
    Trajectory traj;

    public GoToPosition(SampleMecanumDrive drive, Trajectory traj){
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void init() {
        drive.followTrajectoryAsync(traj);
    }

    @Override
    public void update() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
