package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.Command;

public class IntakeMove extends Command {

    int ticks;
    double armPosition;
    double spinPosition;
    public IntakeMove(int ticks, double armPosition, double spinPosition){
        this.ticks = ticks;
        this.armPosition = armPosition;
        this.spinPosition = spinPosition;
    }

    @Override
    public void init() {
        return;
    }

    @Override
    public void update() {
        Tom.intake.moveToPosition(ticks);
        Tom.arm.moveToPosition(armPosition);
        Tom.claw.setSpinPosition(spinPosition);
    }

    @Override
    public boolean isFinished() {
        return Tom.intake.isFinished()&&Tom.arm.isFinished();
    }
}
