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

public class AutoStackTeleOp extends SequentialCommand {

    public AutoStackTeleOp(){
        super(
                new RunCommand(()->Tom.claw.close()),
                new Wait(900),
                new Collect(),
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.TOP.getTicks()),
                        new SequentialCommand(
                                new Wait(150),
                                new RunCommand(()-> Tom.latch.setLatchPosition(Latch.CLOSE))),
                        new IntakeMove(IntakeSlides.TurnValue.RETRACTED.getTicks(),
                                Arm.TurnValue.EXTENDED.getPosition(),
                                Claw.OUT)),
                new Wait(300),
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                        new RunCommand(()->Tom.latch.setLatchPosition(Latch.OPEN))),
                new IntakeMove(IntakeSlides.TurnValue.AUTO_STACK.getTicks(), Arm.TurnValue.EXTENDED.getPosition(), Claw.OUT)
        );
    }
}
