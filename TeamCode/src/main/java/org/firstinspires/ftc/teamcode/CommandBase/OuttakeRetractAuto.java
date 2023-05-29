package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class OuttakeRetractAuto extends Command {

    ElapsedTime stallingTimer;
    boolean extending = false;

    @Override
    public void init() {
        stallingTimer = new ElapsedTime();
        stallingTimer.reset();
    }

    @Override
    public void update() {

//        if(!Tom.outtake.isStalling()){
//            stallingTimer.reset();
//        }


        if(stallingTimer.milliseconds()>2000){
            extending = true;
            Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.TOP.getTicks());
        }else if (!extending){
            Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.RETRACTED.getTicks());
        }

        if(Tom.outtake.isFinished()&&Tom.outtake.getTargetPosition()== OuttakeSlides.TurnValue.TOP.getTicks()){
            extending = false;
            stallingTimer.reset();
        }

    }

    @Override
    public boolean isFinished() {
        return Tom.outtake.isFinished()&&Tom.outtake.getTargetPosition()== OuttakeSlides.TurnValue.RETRACTED.getTicks();
    }
}
