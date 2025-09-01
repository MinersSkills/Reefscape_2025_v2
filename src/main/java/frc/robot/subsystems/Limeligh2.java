package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limeligh2 extends SubsystemBase {

    private final NetworkTable limelightTable;
    private double tx = 0.0;
    private double ty = 0.0;
    private double tid = 0.0;
    public double rotationCommand = 0.0;
    public double erro = 0.0;
    double[] robotPose = new double[6] ;

    public Limeligh2() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Ajusta o robô para alinhar com a AprilTag.
     * 
     * @return O valor de tx, para monitoramento ou depuração.
     */
    public double[] getDadosTag() {

        // Obter o valor de tx da Limelight (deslocamento horizontal)
        tx = limelightTable.getEntry("tx").getDouble(0.0);
        ty = limelightTable.getEntry("ty").getDouble(0.0);
        tid = limelightTable.getEntry("tid").getDouble(0.0);
        robotPose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

        double[] values = new double[6];
        values[0] = tx;
        values[1] = robotPose[5]; //rotação do robô
        values[2] = tid;
        values[3] = ty; 
        // Retornar o valor de tx (deslocamento horizontal)
        return values;
    }

    /**
     * Ajusta o robô para alinhar com a AprilTag.
     * 
     * @return O valor de tx, para monitoramento ou depuração.
     */
    public double getRT() {
        double kp = .009;
        double kd = .0035;

        double vel_rot = getDadosTag()[0] * kp;
        double vel_derivativo = getDadosTag()[0] * kd;
        vel_rot *= Math.PI;

        vel_rot *= -1;

        return vel_rot + vel_derivativo;
    }

    public double getRB_pose() {
        
        double robotRotation = robotPose[5];
        double angleError = robotRotation; 
        double kp = 0.003; 

        double rotationSpeed = angleError * kp;

        return rotationSpeed;
    }

    public Pose2d getLimelightPose() {
        // A Limelight retorna a pose em um array [x, y, z, roll, pitch, yaw]
        double[] botPose = getLimelightBotPose();
        if (botPose.length < 6) {
            return null; // Nenhum dado válido
        }

        return new Pose2d(
            botPose[0], // X
            botPose[1], // Y
            Rotation2d.fromDegrees(botPose[5]) // Yaw
        );
    }

    public double[] getLimelightBotPose() {
        return robotPose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
         
    }

    public double getTimestamp() {
        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        return botpose.length >= 7 ? Timer.getFPGATimestamp() - botpose[6] / 1000.0 : 0;
    }

}
