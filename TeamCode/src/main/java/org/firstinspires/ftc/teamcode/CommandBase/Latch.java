package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.Command;

public class Latch extends Command {

    boolean latchClose;

    public Latch(boolean latchClose){
        this.latchClose = latchClose;
    }

    @Override
    public void init() {
        if(latchClose){
            Tom.latch.setLatchPosition(org.firstinspires.ftc.teamcode.component.Latch.CLOSE);
        }else{
            Tom.latch.setLatchPosition(org.firstinspires.ftc.teamcode.component.Latch.CLOSE);
        }
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
