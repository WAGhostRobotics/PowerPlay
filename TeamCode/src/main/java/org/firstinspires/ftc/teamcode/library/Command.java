package org.firstinspires.ftc.teamcode.library;

/*
    Author: joshi
 */

public abstract class Command {

    public void init(){
        return;
    }
    public abstract void update();
    public abstract boolean isFinished();
}
