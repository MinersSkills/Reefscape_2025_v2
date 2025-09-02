package frc.robot.commands.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;

public class CoralStop extends Command {

    public CoralStop(){
        System.out.println("Movendo construido");
    }

    @Override
    public void initialize(){
        System.out.println("Movendo elevador para baixo");
    }
    
    @Override
    public void execute(){
        OperatorConstants.motorGarra.disable();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
