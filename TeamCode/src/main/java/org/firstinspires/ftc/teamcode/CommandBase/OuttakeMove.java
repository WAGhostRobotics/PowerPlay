package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class OuttakeMove extends Command {

    int ticks = 0;
    public OuttakeMove(int ticks){
        this.ticks = ticks;
    }


    @Override
    public void update() {
        Tom.outtake.setTargetPosition(ticks);
    }

    @Override
    public boolean isFinished() {
        return Tom.outtake.isFinished()&&Tom.outtake.getTargetPosition()==ticks;
    }
}
