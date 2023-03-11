package org.firstinspires.ftc.teamcode.library;

public abstract class Command {

    public void init(){
        return;
    }
    public abstract void update();
    public abstract boolean isFinished();
}
