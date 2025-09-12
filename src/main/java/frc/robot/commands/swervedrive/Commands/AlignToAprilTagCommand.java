package frc.robot.commands.swervedrive.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimeLightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToAprilTagCommand extends Command {
    private final SwerveSubsystem swerve;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    private boolean isRight;
    private double tagid = -1;
    private Timer dontSeeTagTimer, stopTimer;

    public AlignToAprilTagCommand(SwerveSubsystem swerve, boolean isRightScore) {
        xController = new PIDController(Constants.AutonConstants.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
        yController = new PIDController(Constants.AutonConstants.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horitontal movement
        rotController = new PIDController(Constants.AutonConstants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
        this.swerve = swerve;
        this.isRight = isRightScore;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(isRight ? Constants.AutonConstants.ROT_SETPOINT_REEF_ALIGNMENT : -2);
        rotController.setTolerance(Constants.AutonConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(Constants.AutonConstants.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(Constants.AutonConstants.X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(isRight ? Constants.AutonConstants.Y_SETPOINT_REEF_ALIGNMENT
                : -Constants.AutonConstants.Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(Constants.AutonConstants.Y_TOLERANCE_REEF_ALIGNMENT);

        tagid = LimeLightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        if (LimeLightHelpers.getTV("") && LimeLightHelpers.getFiducialID("") == tagid) {
            this.dontSeeTagTimer.reset();

            double[] postions = LimeLightHelpers.getBotPose_TargetSpace("");
            SmartDashboard.putNumber("x", postions[2]);
            double xSpeed = xController.calculate(postions[2]);
            double ySpeed = -yController.calculate(postions[0]);
            double rotValue = -rotController.calculate(postions[4]);

            swerve.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

            if (!rotController.atSetpoint() ||
                    !yController.atSetpoint() ||
                    !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            swerve.drive(new Translation2d(), 0, false);
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public boolean isFinished() {
        return this.dontSeeTagTimer.hasElapsed(Constants.AutonConstants.DONT_SEE_TAG_WAIT_TIME)
                || stopTimer.hasElapsed(Constants.AutonConstants.POSE_VALIDATION_TIME);
    }
}
