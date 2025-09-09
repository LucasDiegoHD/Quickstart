package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;

    public VisionSubsystem(HardwareMap hardwareMap) {
        // Inicializa a Limelight a partir do hardware map.
        // "limelight" deve ser o nome configurado no seu robô.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start(); // Inicia a coleta de dados da câmera
    }

    /**
     * Retorna o desvio horizontal (ângulo) do alvo em graus.
     * Valor negativo significa que o alvo está à esquerda, positivo à direita.
     * @return um Optional contendo o valor de 'tx', ou vazio se nenhum alvo for válido.
     */
    public Optional<Double> getTargetTx() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTx());
        }
        return Optional.empty();
    }

    /**
     * Retorna o desvio vertical (ângulo) do alvo em graus.
     * Pode ser usado como um indicador de distância.
     * @return um Optional contendo o valor de 'ty', ou vazio se nenhum alvo for válido.
     */
    public Optional<Double> getTargetTy() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTy());
        }
        return Optional.empty();
    }

    /**
     * Verifica se a Limelight tem um alvo válido.
     * @return true se um alvo válido for detectado, false caso contrário.
     */
    public boolean hasTarget() {
        return latestResult!= null && latestResult.isValid();
    }

    @Override
    public void periodic() {
        // Este método é chamado automaticamente pelo CommandScheduler a cada ciclo.
        // Atualizamos o resultado mais recente da Limelight aqui.
        latestResult = limelight.getLatestResult();
    }
}