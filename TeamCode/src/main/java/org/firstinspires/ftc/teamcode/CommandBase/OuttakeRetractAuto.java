package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.Command;

public class OuttakeRetractAuto extends Command {


    @Override
    public void update() {
        if(Tom.outtake.isStalling()){
            Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.RETRACTED.getTicks());
        }else{
            Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.TOP.getTicks());
        }

    }

    @Override
    public boolean isFinished() {
        return Tom.outtake.isFinished()&&Tom.outtake.getTargetPosition()== OuttakeSlides.TurnValue.RETRACTED.getTicks();
    }
}
