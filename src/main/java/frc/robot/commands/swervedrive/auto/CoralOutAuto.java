package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;

public class CoralOutAuto extends Command {

    private double speed;

    public CoralOutAuto(double speed){
        this.speed = speed;
    }

    @Override
    public void initialize(){
        System.out.println("Movendo elevador para baixo");
    }
    
    @Override
    public void execute(){
        OperatorConstants.motorGarra.set(speed);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
