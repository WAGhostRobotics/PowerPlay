package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.Arm;
import org.firstinspires.ftc.teamcode.component.Claw;
import org.firstinspires.ftc.teamcode.component.IntakeSlides;
import org.firstinspires.ftc.teamcode.component.Latch;
import org.firstinspires.ftc.teamcode.component.OuttakeSlides;
import org.firstinspires.ftc.teamcode.core.Tom;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;

public class PlaceConeAuto extends SequentialCommand {

    public PlaceConeAuto(double coneArmPosition, int intakeSlidesPosition, double spinPosition){
        super(
                new ParallelCommand(
                        new OuttakeMove(OuttakeSlides.TurnValue.TOP.getTicks()),
                        new SequentialCommand(
                                new Wait(150),
                                new RunCommand(()-> Tom.latch.setLatchPosition(Latch.CLOSE))),
                        new IntakeMove(IntakeSlides.TurnValue.ALMOST_DONE.getTicks(),
                                coneArmPosition,
                                spinPosition, false)),
                new ParallelCommand(
                        new Wait(450),
                        new OuttakeMove(OuttakeSlides.TurnValue.TOP.getTicks())
                ),
                new ParallelCommand(
                        new OuttakeRetractAuto(),
                        new RunCommand(()->Tom.latch.setLatchPosition(Latch.OPEN))),
                new SpecialIntakeCommand(intakeSlidesPosition, coneArmPosition, spinPosition),
                new RunCommand(()->Tom.claw.setClawPosition(Claw.CLOSE)),
                new Wait(350),
                new IntakeMove(intakeSlidesPosition, Arm.TurnValue.LOW.getPosition(), spinPosition),
                new Collect()
        );
    }
}
