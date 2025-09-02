package frc.robot.commands.swervedrive.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;

public class CoralOut extends Command {

    private double speed;

    public CoralOut(double speed){
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
        return false;
    }

}
