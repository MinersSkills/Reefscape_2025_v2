package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;

public class CoralStopAuto extends Command {

    public CoralStopAuto(){
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
        return true;
    }

}
