package org.firstinspires.ftc.teamcode.library;

public class RunCommand extends Command{


    public RunCommand(RunFunction function){
        function.run();
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

