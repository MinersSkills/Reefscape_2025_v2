package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

public class ElevadorDownAuto extends Command {
    
    private final Elevador elevador;
    private final double setpoint;

    public ElevadorDownAuto(Elevador elevador, double setpoint){
        this.elevador = elevador;
        this.setpoint = setpoint;
        addRequirements(elevador);
    }

    @Override
    public void initialize(){
        System.out.println("Movendo elevador para baixo");
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
