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

public class Elevador extends SubsystemBase{
    private SparkMaxConfig motor_elevadorConf;
    private RelativeEncoder encoder_elevador;
    // private Garra garra; 

    private XboxController controller2; // Controle do Xbox
    
    private boolean botao_Quadrado_aux = false; // Sobe braço L4

    private boolean bootao_O_aux = false; // Desce braço

    public Elevador() {
        motor_elevadorConf = new SparkMaxConfig();
        motor_elevadorConf.encoder.positionConversionFactor(1);
        motor_elevadorConf.inverted(true);
        motor_elevadorConf.idleMode(IdleMode.kCoast);

        encoder_elevador = OperatorConstants.motorelevador.getEncoder();

        controller2 = new XboxController(1);

        motor_elevadorConf.closedLoop.pid(0.09, 0, 0.005);

        OperatorConstants.motorelevador.configure(motor_elevadorConf, null, PersistMode.kNoPersistParameters);

        // encoder_elevador.setPosition(0.0);
        // garra = new Garra();

    }

    public void setReferencia(double setpoint) {
        OperatorConstants.motorelevador.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    }

    public void setElevador() {
        boolean botaoOativo = controller2.getBButton(); // Descer Garra e elevador
        boolean botao_Quadrado_ativo = controller2.getXButton(); //L4

        if(botao_Quadrado_ativo && !botao_Quadrado_aux){
            setReferencia(108.0);
            // garra.setReferencia(52.0);
        }
        else if (botaoOativo && !bootao_O_aux) {
            // Quando o botão "O" é pressionado
            // garra.setReferencia(0.0);
            setReferencia(0.1);

        }

        botao_Quadrado_aux = botao_Quadrado_ativo;
        bootao_O_aux = botaoOativo;

    }

    public void dashboard() {
        SmartDashboard.putNumber("Posicao Atual motor elevador: ", encoder_elevador.getPosition());
    }

}