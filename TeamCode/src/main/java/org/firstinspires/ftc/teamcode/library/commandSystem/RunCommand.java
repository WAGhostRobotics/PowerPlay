package org.firstinspires.ftc.teamcode.library.commandSystem;

public class RunCommand extends Command {

    RunFunction func;

    public RunCommand(RunFunction function){
        this.func = function;
    }

    @Override
    public void init() {
        func.run();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

