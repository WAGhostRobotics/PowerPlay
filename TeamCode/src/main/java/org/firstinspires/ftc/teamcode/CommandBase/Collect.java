package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.Latch;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.RunCommand;
import org.firstinspires.ftc.teamcode.library.SequentialCommand;

public class Collect extends SequentialCommand {

    public Collect(){
        super(
                new ParallelCommand(
                        new RunCommand(()->Tom.latch.setLatchPosition(Latch.OPEN)),
                        new IntakeMove(IntakeSlides.TurnValue.RETRACTED.getTicks(), Arm.TurnValue.PARTIAL.getPosition(), Claw.IN),
                        new OuttakeMove(OuttakeSlides.TurnValue.SUPER_RETRACTED.getTicks())),
                new RunCommand(()->Tom.outtake.setTargetPosition(OuttakeSlides.TurnValue.RETRACTED.getTicks())),
                new Wait(100),
                new IntakeMove(IntakeSlides.TurnValue.PLACE_CONE.getTicks(), Arm.TurnValue.RETRACTED.getPosition(), Claw.IN),
                new RunCommand(()-> Tom.claw.open()),
                new Wait(150),
                new IntakeMove(IntakeSlides.TurnValue.PLACE_CONE.getTicks(), Arm.TurnValue.PARTIAL.getPosition(), Claw.IN)
        );
    }
}
