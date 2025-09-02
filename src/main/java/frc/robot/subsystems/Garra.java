package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Garra extends SubsystemBase {
    private SparkMaxConfig motor_bracoConf;
    public RelativeEncoder encoder_braco;

    private SparkMaxConfig motor_garraConf;
    public RelativeEncoder encoder_garra;

    private XboxController controller;

    private boolean bootao_x_aux = false; // Sobe braço L2 e L3

    private boolean bootao_triangulo_aux = false; // Sobe braço L4

    public boolean botao_rb_prev = false;

    public boolean botao_RT_aux = false;

    public double error = 0.0;
    public double vel = 0.0;
    private double DEADZONE = 0.5;

    public Garra() {
        motor_bracoConf = new SparkMaxConfig();
        motor_bracoConf.encoder.positionConversionFactor(1);
        motor_bracoConf.closedLoop.positionWrappingEnabled(false);
        motor_bracoConf.closedLoop.outputRange(-0.35, 0.85);
        motor_bracoConf.inverted(false);
        motor_bracoConf.idleMode(IdleMode.kBrake);

        motor_garraConf = new SparkMaxConfig();

        motor_garraConf.inverted(true);
        motor_garraConf.idleMode(IdleMode.kCoast);

        encoder_braco = OperatorConstants.motorbraco.getEncoder();
        encoder_garra = OperatorConstants.motorGarra.getEncoder();

        controller = new XboxController (1);

        OperatorConstants.motorbraco.configure(motor_bracoConf, null, PersistMode.kNoPersistParameters);
        OperatorConstants.motorGarra.configure(motor_garraConf, null, PersistMode.kNoPersistParameters);

        encoder_braco.setPosition(0.0);
        // encoder_garra.setPosition(0.0);

    }

    public void setReferencia(double setpoint) {
        double currentPosition = encoder_braco.getPosition(); // Pegue a posição atual do motor

        if (setpoint < 0) {
            System.out.println("MENOR");
            motor_bracoConf.closedLoop.p(0.087);
            OperatorConstants.motorbraco.configure(motor_bracoConf, null, PersistMode.kNoPersistParameters);
            OperatorConstants.motorbraco.getClosedLoopController().setReference(setpoint, ControlType.kPosition);

            if (Math.abs(setpoint - currentPosition) < DEADZONE) {
                // Se a diferença for menor que a deadzone, não faz nada
                OperatorConstants.motorbraco.disable();
                return;
            }

        } else {
            motor_bracoConf.closedLoop.pid(0.15, 0, 0);
            OperatorConstants.motorbraco.configure(motor_bracoConf, null, PersistMode.kNoPersistParameters);
            OperatorConstants.motorbraco.getClosedLoopController().setReference(setpoint, ControlType.kPosition);

        }
    }

    public void setBraco() {
        boolean botaoXativo = controller.getAButton(); // Botão X
        boolean botaotrianguloativo = controller.getYButton(); // Botão Triângulo
        boolean botao_RT_ativo = controller.getStartButton();

        if (botaoXativo && !bootao_x_aux) {
            setReferencia(23.3);
            // setReferenciaGarr(posicaoGarra);

        } else if (botaotrianguloativo && !bootao_triangulo_aux) {
            setReferencia(0);
            // setReferenciaGarr(-3);

        }
        else if(botao_RT_ativo && !botao_RT_aux){
            setReferencia(16);
        }

        // Atualiza os estados auxiliares dos botões
        bootao_x_aux = botaoXativo;
        bootao_triangulo_aux = botaotrianguloativo;
        botao_RT_aux = botao_RT_ativo;
    }

    public void setGarra() {
        boolean botao_LB_ativo = controller.getLeftBumperButton(); // Desce garra
        boolean botaoRBativo = controller.getRightBumperButton();
        

        if (botao_LB_ativo) {
            OperatorConstants.motorGarra.set(0.3);

        }
        else if(botaoRBativo){
            OperatorConstants.motorGarra.set(-0.7);
        }
        else {
            OperatorConstants.motorGarra.stopMotor();

        }

    }

    public void dashboard() {
        SmartDashboard.putNumber("Posicao Atual motor braco: ", encoder_braco.getPosition());
        SmartDashboard.putNumber("Posicao Atual motor garra: ", encoder_garra.getPosition());
    }

}