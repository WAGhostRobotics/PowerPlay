package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.Command;

public class Wait extends Command {

    int millis;
    ElapsedTime timer;

    public Wait(int millis){
        this.millis = millis;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void update() {
        return;
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds()>millis;
    }
}
