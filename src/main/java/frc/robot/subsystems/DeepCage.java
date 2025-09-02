package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class DeepCage extends SubsystemBase {

    private RelativeEncoder encoder_cage;
    private SparkMaxConfig  motor_cageConfig;

    private PS4Controller controller;
    boolean bootao_x_aux = false; // Sobe braço L2 e L3
    boolean bootao_triangulo_aux = false; // Sobe braço L4

    public DeepCage(){
        controller = new PS4Controller(0);
        motor_cageConfig = new SparkMaxConfig();
        motor_cageConfig.encoder.positionConversionFactor(1);
        motor_cageConfig.inverted(false);
        motor_cageConfig.idleMode(IdleMode.kBrake);

        encoder_cage = OperatorConstants.motorCage.getEncoder();
        encoder_cage.setPosition(0.0);
    }

    public void setCage(){
        boolean botaoXativo = controller.getCrossButton(); // Botão X
        boolean botaotrianguloativo = controller.getTriangleButton(); // Botão Triângulo

        if (botaoXativo) {
            OperatorConstants.motorCage.set(0.5);

        } else if (botaotrianguloativo) {
            OperatorConstants.motorCage.set(-0.5);

        }
        else if(!botaoXativo || !botaotrianguloativo){
            OperatorConstants.motorCage.disable();
        }

        // Atualiza os estados auxiliares dos botões
        bootao_x_aux = botaoXativo;
        bootao_triangulo_aux = botaotrianguloativo;
    }
}
