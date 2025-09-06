package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Action action;
    private final double targetVelocity;

    public enum Action {
        SPIN_UP,
        STOP
    }

    public ShootCommand(ShooterSubsystem shooter, Action action, double targetVelocity) {
        this.shooter = shooter;
        this.action = action;
        this.targetVelocity = targetVelocity;
        addRequirements(shooter);
    }

    public ShootCommand(ShooterSubsystem shooter, Action action) {
        this(shooter, action, 0);
    }

    @Override
    public void initialize() {
        switch (action) {
            case SPIN_UP:
                shooter.spinUp(targetVelocity);
                break;
            case STOP:
                shooter.stop();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // A ação de parar é instantânea.
        if (action == Action.STOP) {
            return true;
        }
        // Para SPIN_UP, o comando termina quando atinge a velocidade.
        // Útil para sequências autônomas.
        return shooter.isAtTargetVelocity(targetVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        // Se o comando for interrompido, para o motor por segurança.
        if (interrupted) {
            shooter.stop();
        }
    }
}