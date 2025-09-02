package frc.robot.commands.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Garra;

public class IntakeCoral extends SequentialCommandGroup {

    private final Elevador elevador;
    private final Garra garra;

    public IntakeCoral(Elevador elevador, Garra garra) {
        this.elevador = elevador;
        this.garra = garra;
        addRequirements(elevador, garra);

        addCommands(
            new ElevadorUP(elevador, 32.2).withTimeout(.1),
            new MoveBracoDown(garra, -5.6).withTimeout(.35),
            Commands.parallel(
                            new CoralOut(-0.45).withTimeout(1.1),
                            new WaitCommand(0.05),
                            new ElevadorDown(elevador, 24.8)
                                            .withTimeout(.35)),

            new ElevadorUP(elevador, 36.2).withTimeout(.35),
            new MoverBracoUP(garra, 23.3).withTimeout(.35),
            new ElevadorUP(elevador, 1).withTimeout(.35),
            new CoralStop().withTimeout(.1));

    }

}
