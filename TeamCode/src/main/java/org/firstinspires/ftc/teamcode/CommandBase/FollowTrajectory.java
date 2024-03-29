package org.firstinspires.ftc.teamcode.CommandBase;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class FollowTrajectory extends Command {

    SampleMecanumDrive drive;
    Trajectory traj;

    public FollowTrajectory(SampleMecanumDrive drive, Trajectory traj){
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void init() {
        drive.followTrajectoryAsync(traj);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
