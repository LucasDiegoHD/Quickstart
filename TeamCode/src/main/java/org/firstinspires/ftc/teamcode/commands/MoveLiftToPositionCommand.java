// Ficheiro: commands/lift/MoveLiftToPositionCommand.java
package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class MoveLiftToPositionCommand extends CommandBase {

    private final LiftSubsystem lift;
    private final int targetPosition;

    public MoveLiftToPositionCommand(LiftSubsystem lift, int targetPosition) {
        this.lift = lift;
        this.targetPosition = targetPosition;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.goToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return!lift.isBusy();
    }
}