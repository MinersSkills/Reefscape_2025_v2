package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Garra;

public class MoveBracoDownAuto extends Command {

    private final Garra bracoGarra;
    private final double setpoint;
    
    public MoveBracoDownAuto(Garra bracoGarra, double setpoint){
        this.bracoGarra = bracoGarra;
        this.setpoint = setpoint;
        addRequirements(bracoGarra);
    }

    @Override
    public void initialize(){
        System.out.println("Descendo Braço");
    }

    @Override
    public void execute(){
        bracoGarra.setReferencia(setpoint);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
