package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;

public class IntakeAlga {

    public SparkMaxConfig motor_intakeAlga_Config;
    public SparkMaxConfig motor_intakeAlga_eixo_config;
    public RelativeEncoder encoder_intakeAlga;

    public PS4Controller controller;

    private boolean botao_Quadrado_aux = false;

    private boolean bootao_O_aux = false;

    public boolean bootao_trian_aux = false;
    public boolean botao_cross_aux = false;
    public boolean botao_L3_aux = false;
    private double DEADZONE = 0.5;
    public Timer timer;

    public IntakeAlga() {

        controller = new PS4Controller(1);
        timer = new Timer();
        motor_intakeAlga_Config = new SparkMaxConfig();
        motor_intakeAlga_eixo_config = new SparkMaxConfig();

        motor_intakeAlga_Config.encoder.positionConversionFactor(1);
        motor_intakeAlga_Config.closedLoop.positionWrappingEnabled(false);
        motor_intakeAlga_Config.closedLoop.outputRange(-0.2, 0.3);
        motor_intakeAlga_Config.inverted(true);
        motor_intakeAlga_Config.idleMode(IdleMode.kBrake);
        motor_intakeAlga_eixo_config.idleMode(IdleMode.kBrake);

        encoder_intakeAlga = OperatorConstants.motorIntakeAlga.getEncoder();

        OperatorConstants.motorIntakeAlga.configure(motor_intakeAlga_Config, null, PersistMode.kNoPersistParameters);

    }

    public void setReferencia(double setpoint) {
        double currentPosition = encoder_intakeAlga.getPosition(); // Pegue a posição atual do motor

        if (setpoint < 1) {
            System.out.println("MENOR");
            motor_intakeAlga_Config.idleMode(IdleMode.kBrake);
            motor_intakeAlga_Config.closedLoop.p(0.02);
            OperatorConstants.motorIntakeAlga.configure(motor_intakeAlga_Config, null, PersistMode.kNoPersistParameters);
            OperatorConstants.motorIntakeAlga.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
            OperatorConstants.motorIntakeAlga.stopMotor();

            if (Math.abs(setpoint - currentPosition) < DEADZONE) {
                // Se a diferença for menor que a deadzone, não faz nada
                OperatorConstants.motorIntakeAlga.disable();
                return;
            }

        } else {
            motor_intakeAlga_Config.idleMode(IdleMode.kBrake);
            motor_intakeAlga_Config.closedLoop.pid(.06, 0, 0);
            OperatorConstants.motorIntakeAlga.configure(motor_intakeAlga_Config, null, PersistMode.kNoPersistParameters);
            OperatorConstants.motorIntakeAlga.getClosedLoopController().setReference(setpoint, ControlType.kPosition);

        }
    }

    public void setIntakeAlga() {
        boolean botaoOativo =  controller.getShareButton();
        boolean botao_Quadrado_ativo = controller.getTouchpadButton();

        if (botao_Quadrado_ativo && !botao_Quadrado_aux) {
            setReferencia(3.5);
            OperatorConstants.motorExioIntakeAlga.set(-0.2);
            timer.start();
        } else if (botaoOativo && !bootao_O_aux) {
            OperatorConstants.motorExioIntakeAlga.set(0.2);
            timer.start();
        } else if(timer.get() > 2.5) {
            setReferencia(1);
            OperatorConstants.motorExioIntakeAlga.stopMotor();
            timer.reset();
            timer.stop();
        }

        botao_Quadrado_aux = botao_Quadrado_ativo;
        bootao_O_aux = botaoOativo;

    }

    public void eixoAlga() {
        boolean botaotrianativo = controller.getTriangleButton(); 
        boolean botaoXXativo = controller.getCrossButton();

        if (botaotrianativo) {

        }
        else if(botaoXXativo){

        }

        bootao_trian_aux = botaotrianativo;
        botao_cross_aux = botaoXXativo;

    }

    public void dashboard() {
        SmartDashboard.putNumber("Posicao Atual motor intake Alga: ", encoder_intakeAlga.getPosition());
    }

}
