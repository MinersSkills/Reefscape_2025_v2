// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.Commands.CoralOut;
import frc.robot.commands.swervedrive.Commands.CoralStop;
import frc.robot.commands.swervedrive.Commands.ElevadorDown;
import frc.robot.commands.swervedrive.Commands.ElevadorUP;
import frc.robot.commands.swervedrive.Commands.IntakeCoral;
import frc.robot.commands.swervedrive.Commands.MoveBracoDown;
import frc.robot.commands.swervedrive.Commands.MoverBracoUP;
import frc.robot.commands.swervedrive.auto.CoralOutAuto;
import frc.robot.commands.swervedrive.auto.CoralStopAuto;
import frc.robot.commands.swervedrive.auto.ElevadorDownAuto;
import frc.robot.commands.swervedrive.auto.ElevadorUPAuto;
import frc.robot.commands.swervedrive.auto.MoveBracoDownAuto;
import frc.robot.commands.swervedrive.auto.MoveToAprilTag;
import frc.robot.subsystems.DeepCage;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Garra;
import frc.robot.subsystems.IntakeAlga;
import frc.robot.subsystems.Limeligh2;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        SendableChooser<Command> autoChooser;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        final CommandPS4Controller drivePS4 = new CommandPS4Controller(0);
        // The robot's subsystems and commands are defined here...
        public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));
        public final Garra garra = new Garra();
        public final Elevador elevador = new Elevador();
        public final IntakeAlga intakeAlga = new IntakeAlga();
        public final Limeligh2 limelight = new Limeligh2();
        public final DeepCage deepCage = new DeepCage();

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> drivePS4.getLeftY(),
                        () -> drivePS4.getLeftX())
                        .withControllerRotationAxis(drivePS4::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(.9)
                        .scaleRotation(.4)
                        .allianceRelativeControl(false);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(drivePS4::getRightX,
                        drivePS4::getRightY)
                        .headingWhile(true);

        SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -drivePS4.getLeftY(),
                        () -> -drivePS4.getLeftX())
                        .withControllerRotationAxis(() -> drivePS4.getRawAxis(2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.85)
                        .allianceRelativeControl(true);

        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        drivePS4.getRawAxis(
                                                        2) * Math.PI)
                                        * (Math.PI * 2),
                                        () -> Math.cos(
                                                        drivePS4.getRawAxis(
                                                                        2) * Math.PI)
                                                        *
                                                        (Math.PI * 2))
                        .headingWhile(true);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandPS4Controller
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         * 
         * 
         * 
         * 
         */
        private void configureBindings() {

                NamedCommands.registerCommand("test", Commands.print("I EXIST"));

                NamedCommands.registerCommand("Rotacionar", drivebase.AjusteComTX(0.2, 0.1));
                NamedCommands.registerCommand("AjusteAprilTag", drivebase.alignToAprilTagInFieldRight(-17.5, 0.1));
                NamedCommands.registerCommand("FrenteTY", drivebase.AjusteComTY(.5, 0.1));
                NamedCommands.registerCommand("TrasTY", drivebase.AjusteComTY_Tras(3, 0.1));        

                NamedCommands.registerCommand("AjusteAprilTag_Red", drivebase.alignToAprilTagInFieldRight(-17.5, 0.1));
                NamedCommands.registerCommand("FrenteTY_Red", drivebase.AjusteComTY(-5.35, 0.1));
                NamedCommands.registerCommand("TrasTY_Red", drivebase.AjusteComTY(-6.8, 0.1));

                NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(elevador, garra));

                NamedCommands.registerCommand("ElevadorUP", new ElevadorUPAuto(elevador, 108));
                NamedCommands.registerCommand("ElevadorDown", new ElevadorDownAuto(elevador, 0.1));
                NamedCommands.registerCommand("GarraDown", new MoveBracoDownAuto(garra, 17.5));
                NamedCommands.registerCommand("GarraPos0", new MoveBracoDownAuto(garra, 0.1));
 
                NamedCommands.registerCommand("CoralOut", new CoralOutAuto(.35));
                NamedCommands.registerCommand("CoralStop", new CoralStopAuto());


                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                // drivebase.PoseEstimator();

                if (RobotBase.isSimulation()) {
                        // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
                } else {
                        // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                if (Robot.isSimulation()) {
                        // drivePS4.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new
                        // Pose2d(3, 3, new Rotation2d()))));
                        // drivePS4.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

                } else {

                        drivePS4.R3().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        drivePS4.options().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

                        // Os seguintes commands são usadas para o lado direito do reef
                        drivePS4.R2().onTrue( // L4
                                        Commands.sequence(
                                                        drivebase.zerarOdometria(),
                                                        drivebase.MoverRobo(.32, 0.1),
                                                        drivebase.AlinharoboRight(0, 0.1),
                                                        drivebase.alignToAprilTagInFieldRight(-20.5, 0.1),
                                                        new MoveBracoDown(garra, 17.5).withTimeout(0.3)

                                        // Commands.parallel(
                                        // new MoverBracoUP(garra, 22.3).withTimeout(.7),
                                        // new ElevadorUP(elevador, 108)
                                        // .withTimeout(.9)),
                                        // new WaitCommand(.3),
                                        // drivebase.MoverRobo(.125, 0.03),
                                        // new MoveBracoDown(garra, 17.5).withTimeout(0.3),
                                        // new CoralOut(0.35).withTimeout(0.25),
                                        // new CoralStop().withTimeout(.1)
                                        ));

                        drivePS4.R1().onTrue(// L3
                                        Commands.sequence(
                                                        drivebase.AlinharoboRight(0, 0.2).withTimeout(.1),
                                                        drivebase.zerarOdometria(),
                                                        drivebase.MoverRobo(.28, 0.03),
                                                        drivebase.alignToAprilTagInFieldRight(-24, 0.1),
                                                        new MoveBracoDown(garra, 17).withTimeout(.3),
                                                        new CoralOut(0.35).withTimeout(0.25),
                                                        new CoralStop().withTimeout(.1)));

                        drivePS4.circle().onTrue(// L2
                                        Commands.sequence(
                                                        drivebase.AlinharoboRight(0, 0.2).withTimeout(.5),
                                                        new MoverBracoUP(garra, 16).withTimeout(0.35),
                                                        drivebase.zerarOdometria(),
                                                        drivebase.MoverRobo(.25, 0.05),
                                                        drivebase.alignToAprilTagInFieldRight(-23.8, 0.1),
                                                        drivebase.MoverRobo(.2, 0.1),
                                                        new MoveBracoDown(garra, 11).withTimeout(.6),
                                                        new CoralOut(0.35).withTimeout(0.25),
                                                        new CoralStop().withTimeout(.1)));

                        // Os seguintes commands serão usados no lado esquerdo do reef

                        drivePS4.L1().onTrue( // L3
                                        Commands.sequence(
                                                        drivebase.AlinharoboRight(0, 0.2).withTimeout(.1),
                                                        drivebase.zerarOdometria(),
                                                        drivebase.MoverRobo(.28, 0.03),
                                                        drivebase.alignToAprilTagInFieldRight(17, 0.1),
                                                        new MoveBracoDown(garra, 17).withTimeout(.3),
                                                        new CoralOut(0.35).withTimeout(0.25),
                                                        new CoralStop().withTimeout(.1)));

                        drivePS4.square().onTrue(// L2
                                        Commands.sequence(
                                                        drivebase.AlinharoboRight(0, 0.2).withTimeout(.5),
                                                        new MoverBracoUP(garra, 16).withTimeout(0.35),
                                                        drivebase.zerarOdometria(),
                                                        drivebase.MoverRobo(.25, 0.05),
                                                        drivebase.alignToAprilTagInFieldRight(21, 0.1),
                                                        drivebase.MoverRobo(.2, 0.1),
                                                        new MoveBracoDown(garra, 11).withTimeout(.6),
                                                        new CoralOut(0.35).withTimeout(0.25),
                                                        new CoralStop().withTimeout(.1)));

                        drivePS4.L2().onTrue( // L4
                                        Commands.sequence(
                                                drivebase.zerarOdometria(),
                                                drivebase.MoverRobo(.32, 0.1),
                                                drivebase.AlinharoboRight(0, 0.1).withTimeout(0.2),
                                                drivebase.alignToAprilTagInFieldRight(20.5, 0.1),
                                                new MoveBracoDown(garra, 17.5).withTimeout(0.3)

                                // Commands.parallel(
                                // new MoverBracoUP(garra, 22.3).withTimeout(.7),
                                // new ElevadorUP(elevador, 108)
                                // .withTimeout(.9)),
                                // new WaitCommand(.3),
                                // drivebase.MoverRobo(.125, 0.03),
                                // new MoveBracoDown(garra, 17.5).withTimeout(0.3),
                                // new CoralOut(0.35).withTimeout(0.25),
                                // new CoralStop().withTimeout(.1)
                                        ));

                        // Intake !!!!
                        drivePS4.L3().onTrue(
                                        Commands.sequence(
                                                        new ElevadorUP(elevador, 32.6).withTimeout(.1),
                                                        new MoveBracoDown(garra, -5.6).withTimeout(.35),
                                                        Commands.parallel(
                                                                        new CoralOut(-0.4).withTimeout(1.1),
                                                                        new WaitCommand(0.1),
                                                                        new ElevadorDown(elevador, 24.8)
                                                                                        .withTimeout(.35)),

                                                        new ElevadorUP(elevador, 36.2).withTimeout(.35),
                                                        new MoverBracoUP(garra, 23.5).withTimeout(.35),
                                                        new ElevadorUP(elevador, 1).withTimeout(.35),
                                                        new CoralStop().withTimeout(.1)));
                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
