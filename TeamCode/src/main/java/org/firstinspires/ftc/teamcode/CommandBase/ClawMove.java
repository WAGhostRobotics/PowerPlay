package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.Command;

public class ClawMove extends Command {

    double clawPosition;
    ElapsedTime timer;
    static int millis = 300;

    public ClawMove(double clawPosition){
        this.clawPosition = clawPosition;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        timer.reset();
        Tom.claw.setClawPosition(clawPosition);
    }

    @Override
    public void update() {
        return;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds()>300;
    }
}
