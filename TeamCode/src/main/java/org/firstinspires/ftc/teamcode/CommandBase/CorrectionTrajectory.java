package org.firstinspires.ftc.teamcode.CommandBase;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class CorrectionTrajectory extends Command {

    SampleMecanumDrive drive;
    Trajectory traj;
    Pose2d goToConePose;

    public CorrectionTrajectory(SampleMecanumDrive drive, Pose2d goToConePose){
        this.drive = drive;
        this.goToConePose = goToConePose;
    }

    @Override
    public void init() {
        traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(goToConePose)
                .build();
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
