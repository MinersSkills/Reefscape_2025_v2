package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Elevador extends SubsystemBase {
    private SparkMaxConfig motor_elevadorConf;
    private RelativeEncoder encoder_elevador;

    private PS4Controller controller2; // Controle do Xbox

    private boolean botao_Quadrado_aux = false; // Sobe braço L4

    private boolean bootao_O_aux = false; // Desce braço

    public Elevador() {
        motor_elevadorConf = new SparkMaxConfig();
        motor_elevadorConf.encoder.positionConversionFactor(1);
        motor_elevadorConf.inverted(true);
        motor_elevadorConf.idleMode(IdleMode.kCoast);

        encoder_elevador = OperatorConstants.motorelevador.getEncoder();

        controller2 = new PS4Controller(0);

        motor_elevadorConf.closedLoop.pid(0.17, 0, 0.004);

        OperatorConstants.motorelevador.configure(motor_elevadorConf, null, PersistMode.kNoPersistParameters);

        // encoder_elevador.setPosition(0.0);
    }

    public void setReferencia(double setpoint) {
        OperatorConstants.motorelevador.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    }

    public void setElevador() {
        boolean botaoOativo = controller2.getCircleButton(); // Descer Garra e elevador
        boolean botao_Quadrado_ativo = controller2.getCrossButton(); // L4

        if (botao_Quadrado_ativo && !botao_Quadrado_aux) {
            setReferencia(85);
        } else if (botaoOativo && !bootao_O_aux) {
            setReferencia(1);
        }

        botao_Quadrado_aux = botao_Quadrado_ativo;
        bootao_O_aux = botaoOativo;

    }

    public void dashboard() {
        SmartDashboard.putNumber("Posicao Atual motor elevador: ", encoder_elevador.getPosition());
    }

}