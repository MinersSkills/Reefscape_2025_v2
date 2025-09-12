package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;

public class Intake {

    public SparkMaxConfig motor_Intake_Config;
    public SparkMaxConfig motor_Intake_eixo_config;
    public RelativeEncoder encoder_Intake;

    public PS4Controller controller;

    private boolean botao_Quadrado_aux = false;

    private boolean bootao_O_aux = false;

    public boolean bootao_trian_aux = false;
    public boolean botao_cross_aux = false;
    public boolean botao_L3_aux = false;

    private double DEADZONE = 0.5;

    public Intake() {

        controller = new PS4Controller(0);
        motor_Intake_Config = new SparkMaxConfig();
        motor_Intake_eixo_config = new SparkMaxConfig();

        motor_Intake_Config.encoder.positionConversionFactor(1);
        motor_Intake_Config.closedLoop.positionWrappingEnabled(false);
        motor_Intake_Config.closedLoop.outputRange(-0.2, 0.3);
        motor_Intake_Config.closedLoop.pid(0.02, 0, 0);
        motor_Intake_Config.inverted(true);
        motor_Intake_Config.idleMode(IdleMode.kBrake);
        motor_Intake_eixo_config.idleMode(IdleMode.kBrake);

        encoder_Intake = OperatorConstants.motorIntake.getEncoder();
        OperatorConstants.motorIntake.configure(motor_Intake_Config, null, PersistMode.kNoPersistParameters);
    }

    public void setReferencia(double setpoint) {
        double currentPosition = encoder_Intake.getPosition(); // Pegue a posição atual do motor

        if (setpoint < 1) {
            System.out.println("MENOR");
            motor_Intake_Config.idleMode(IdleMode.kBrake);
            motor_Intake_Config.closedLoop.p(0.02);
            OperatorConstants.motorIntake.configure(motor_Intake_Config, null, PersistMode.kNoPersistParameters);
            OperatorConstants.motorIntake.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
            OperatorConstants.motorIntake.stopMotor();

            if (Math.abs(setpoint - currentPosition) < DEADZONE) {
                // Se a diferença for menor que a deadzone, não faz nada
                OperatorConstants.motorIntake.disable();
                return;
            }

        } else {
            motor_Intake_Config.idleMode(IdleMode.kBrake);
            motor_Intake_Config.closedLoop.pid(.06, 0, 0);
            OperatorConstants.motorIntake.configure(motor_Intake_Config, null, PersistMode.kNoPersistParameters);
            OperatorConstants.motorIntake.getClosedLoopController().setReference(setpoint, ControlType.kPosition);

        }
    }

    public void SetPoint(double setpoint) {
        OperatorConstants.motorIntake.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    }

    public void setIntake() {
        boolean botaoOativo = controller.getShareButton();
        boolean botao_Quadrado_ativo = controller.getTouchpadButton();

        if (botao_Quadrado_ativo && !botao_Quadrado_aux) {
            SetPoint(40);
        } else if (botaoOativo && !bootao_O_aux) {
            SetPoint(0);
        }

        botao_Quadrado_aux = botao_Quadrado_ativo;
        bootao_O_aux = botaoOativo;
    }

    public void dashboard() {
        SmartDashboard.putNumber("Posicao Atual motor intake Alga: ", encoder_Intake.getPosition());
    }

}
