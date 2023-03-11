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
        if(timer == null){
            timer = new ElapsedTime();
            timer.reset();
        }
    }

    @Override
    public void update() {
        if(timer == null){
            timer = new ElapsedTime();
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds()>millis;
    }
}
