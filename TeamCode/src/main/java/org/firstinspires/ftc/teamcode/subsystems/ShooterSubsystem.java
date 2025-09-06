package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSubsystem extends SubsystemBase {

    private static final double VELOCITY_TOLERANCE = 20.0; // Ticks por segundo

    private final DcMotorEx shooterMotor;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, TConstants.Shooter.ShooterMotor);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD); // Ajuste conforme necessário
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // NOTA: Estes coeficientes PIDF são exemplos e DEVEM ser afinados.
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10, 3, 0, 12);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    /**
     * Ativa o motor do lançador para atingir uma velocidade alvo.
     * @param targetVelocity A velocidade desejada em ticks por segundo.
     */
    public void spinUp(double targetVelocity) {
        shooterMotor.setVelocity(targetVelocity);
    }

    /**
     * Para completamente o motor do lançador.
     */
    public void stop() {
        shooterMotor.setVelocity(0);
    }

    /**
     * Verifica se o motor atingiu a velocidade alvo dentro de uma tolerância.
     * @param targetVelocity A velocidade alvo a ser verificada.
     * @return true se a velocidade atual estiver dentro da tolerância.
     */
    public boolean isAtTargetVelocity(double targetVelocity) {
        return Math.abs(shooterMotor.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }
}