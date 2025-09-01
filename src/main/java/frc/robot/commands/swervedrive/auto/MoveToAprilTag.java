package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveToAprilTag extends Command {
    
    private final SwerveSubsystem drivebase;


    public MoveToAprilTag(SwerveSubsystem drivebase){
        this.drivebase = drivebase;
       addRequirements(drivebase);

    }

    @Override
    public void initialize(){
        System.out.println("Criando");
    }

    @Override
    public void execute(){
        drivebase.alignToAprilTagInFieldRight(21.9, 0.1);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
