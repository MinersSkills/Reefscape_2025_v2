package frc.robot.commands.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Garra;

public class MoverBracoUP extends Command {
    private final Garra bacoGarra;
    private final double setpoint;

    public MoverBracoUP(Garra bracoGarra, double setpoint){
        this.bacoGarra = bracoGarra;
        this.setpoint = setpoint;
        addRequirements(bracoGarra);
    }

    @Override
    public void initialize(){
        System.out.println("Movendo braço para cima");
    }
    
    @Override
    public void execute(){
        bacoGarra.setReferencia(setpoint);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
