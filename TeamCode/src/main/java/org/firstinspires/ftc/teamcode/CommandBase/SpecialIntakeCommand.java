package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class SpecialIntakeCommand extends Command {

    int ticks = 0;
    double armPosition = 0;
    double spinPosition;

    boolean waitForFinish = true;

    public SpecialIntakeCommand(int ticks, double armPosition, double spinPosition){
        this.ticks = ticks;
        this.armPosition = armPosition;
        this.spinPosition = spinPosition;
    }



    @Override
    public void update() {

        if(Tom.arm.getTargetPosition()==armPosition && Tom.arm.isFinished()){
            Tom.intake.setTargetPosition(ticks);
        }else{
            Tom.intake.setTargetPosition(IntakeSlides.TurnValue.ALMOST_DONE.getTicks());
        }
        Tom.arm.setTargetPosition(armPosition);
        Tom.claw.setSpinPosition(spinPosition);
    }

    @Override
    public boolean isFinished() {
        return (Tom.intake.isFinished()&&Tom.arm.isFinished()&&Tom.arm.getTargetPosition()== armPosition&&Tom.intake.getTargetPosition() == ticks)
                || !waitForFinish;
    }

}
