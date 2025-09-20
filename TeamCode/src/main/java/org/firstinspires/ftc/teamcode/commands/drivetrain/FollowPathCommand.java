// Ficheiro: commands/drivetrain/FollowPathCommand.java
package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class FollowPathCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final PathChain path;

    public FollowPathCommand(DrivetrainSubsystem drivetrain, PathChain path) {
        this.drivetrain = drivetrain;
        this.path = path;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().followPath(path);
    }

    @Override
    public boolean isFinished() {
        return!drivetrain.getFollower().isBusy();
    }
}