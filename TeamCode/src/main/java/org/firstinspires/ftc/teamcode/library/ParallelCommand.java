package org.firstinspires.ftc.teamcode.library;

import java.util.ArrayList;

public class ParallelCommand extends Command{

    ArrayList<Command> commandList;

    public ParallelCommand(ArrayList<Command> commandList){
        this.commandList = commandList;
    }

    @Override
    public void init() {
        for(int i=0;i<commandList.size();i++){
            commandList.get(i).init();
        }
    }

    @Override
    public void update() {
        for(int i=0;i<commandList.size();i++){
            commandList.get(i).update();
        }
    }

    @Override
    public boolean isFinished() {
        int finished = 0;
        for(int i=0;i<commandList.size();i++){
            if(commandList.get(i).isFinished()){
                finished++;
            }
        }

        if(finished==commandList.size()){
            return true;
        }

        return false;
    }

    public ArrayList<Integer> getFinishedIndexes(){
        ArrayList<Integer> finished = new ArrayList<Integer>();
        for(int i=0;i<commandList.size();i++){
            if(commandList.get(i).isFinished()){
                finished.add(i);
            }
        }

        return finished;

    }
}
