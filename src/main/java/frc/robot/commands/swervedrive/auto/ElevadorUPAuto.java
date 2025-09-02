package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

public class ElevadorUPAuto extends Command {
    private final Elevador elevador;
    private final double setpoint;

    public ElevadorUPAuto(Elevador elevador, double setpoint){
        this.elevador = elevador;
        this.setpoint = setpoint;
        addRequirements(elevador);
    }

    @Override
    public void initialize(){
        System.out.println("Movendo elevador para cima");
    }
    
    @Override
    public void execute(){
        elevador.setReferencia(setpoint);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
