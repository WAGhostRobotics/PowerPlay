package org.firstinspires.ftc.teamcode.library;

import java.util.ArrayList;

public class SequentialCommand extends Command{

    ArrayList<Command> commandList;

    private int index;

    private boolean finished = false;

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

            if(commandList.get(index).isFinished()){
                if(index<commandList.size()-1){
                    index++;
                    commandList.get(index).init();

                }else{
                    finished = true;
                }
            }

            commandList.get(index).update();

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    public int getIndex(){
        return index;
    }


}
