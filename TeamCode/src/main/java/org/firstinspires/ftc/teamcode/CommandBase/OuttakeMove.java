package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.Command;

public class OuttakeMove extends Command {

    int ticks;
    public OuttakeMove(int ticks){
        this.ticks = ticks;
    }

    @Override
    public void init() {
        return;
    }

    @Override
    public void update() {
        Tom.outtake.moveToPosition(ticks);
    }

    @Override
    public boolean isFinished() {
        return Tom.outtake.isFinished();
    }
}