package org.firstinspires.ftc.teamcode.library;

import java.util.ArrayList;

public class SequentialCommand extends Command{

    ArrayList<Command> commandList;

    private int index;

    public SequentialCommand(ArrayList<Command> commandList){
        this.commandList = commandList;
        index = 0;
    }

    @Override
    public void init() {
        commandList.get(0).init();
    }

    @Override
    public void update() {
        if(commandList.get(0).isFinished()){
            index++;
            commandList.get(index).init();
        }else{
            commandList.get(index).update();
        }
    }

    @Override
    public boolean isFinished() {
        return commandList.get(commandList.size()-1).isFinished();
    }


}
