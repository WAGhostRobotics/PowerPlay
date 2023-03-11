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

public class PlaceConeAuto extends SequentialCommand {

    public PlaceConeAuto(double coneArmPosition){
        super(
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.TOP.getTicks()),
                        new SequentialCommand(
                                new Wait(200),
                                new RunCommand(()-> Tom.latch.setLatchPosition(Latch.CLOSE))),
                        new IntakeMove(IntakeSlides.TurnValue.ALMOST_DONE.getTicks(),
                                coneArmPosition,
                                Claw.OUT)),
                new Wait(300),
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.RETRACTED.getTicks()),
                        new RunCommand(()->Tom.latch.setLatchPosition(Latch.OPEN))),
                new IntakeMove(IntakeSlides.TurnValue.AUTO_EXTENDED_LEFT.getTicks(), coneArmPosition, Claw.OUT),
                new RunCommand(()->Tom.claw.setClawPosition(Claw.CLOSE)),
                new Wait(350),
                new IntakeMove(IntakeSlides.TurnValue.AUTO_EXTENDED_LEFT.getTicks(), Arm.TurnValue.LOW.getPosition(), Claw.OUT),
                new Collect()
        );
    }
}
