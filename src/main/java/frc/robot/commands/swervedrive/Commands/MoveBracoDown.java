package frc.robot.commands.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Garra;

public class MoveBracoDown extends Command {

    private final Garra braco;
    private final double setpoint;
    
    public MoveBracoDown(Garra braco, double setpoint){
        this.braco = braco;
        this.setpoint = setpoint;
        addRequirements(braco);
    }

    @Override
    public void initialize(){
        System.out.println("Descendo Braço");
    }

    @Override
    public void execute(){
        braco.setReferencia(setpoint);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
