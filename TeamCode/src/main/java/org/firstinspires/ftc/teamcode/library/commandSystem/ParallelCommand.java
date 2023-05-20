package org.firstinspires.ftc.teamcode.library.commandSystem;

import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class ParallelCommand extends Command {

    Command[] commandList;

    public ParallelCommand(Command... commandList){
        this.commandList = commandList;
    }

    @Override
    public void init() {
        for(int i=0;i<commandList.length;i++){
            commandList[i].init();
        }
    }

    @Override
    public void update() {
        for(int i=0;i<commandList.length;i++){
            commandList[i].update();
        }
    }

    @Override
    public boolean isFinished() {
        int finished = 0;
        for(int i=0;i<commandList.length;i++){
            if(commandList[i].isFinished()){
                finished++;
            }
        }

        if(finished==commandList.length){
            return true;
        }

        return false;
    }


}
