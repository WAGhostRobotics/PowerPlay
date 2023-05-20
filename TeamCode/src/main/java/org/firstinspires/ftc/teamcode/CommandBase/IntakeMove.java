package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class IntakeMove extends Command {

    int ticks = 0;
    double armPosition = 0;
    double spinPosition;

    boolean waitForFinish = true;

    public IntakeMove(int ticks, double armPosition, double spinPosition){
        this.ticks = ticks;
        this.armPosition = armPosition;
        this.spinPosition = spinPosition;
    }

    public IntakeMove(int ticks, double armPosition, double spinPosition, boolean waitForFinish){
        this.ticks = ticks;
        this.armPosition = armPosition;
        this.spinPosition = spinPosition;

        this.waitForFinish = waitForFinish;
    }


    @Override
    public void update() {
        Tom.intake.setTargetPosition(ticks);
        Tom.arm.setTargetPosition(armPosition);
        Tom.claw.setSpinPosition(spinPosition);
    }

    @Override
    public boolean isFinished() {
        return (Tom.intake.isFinished()&&Tom.arm.isFinished()&&Tom.arm.getTargetPosition()==armPosition&&Tom.intake.getTargetPosition()==ticks)
                || !waitForFinish;
    }

}
