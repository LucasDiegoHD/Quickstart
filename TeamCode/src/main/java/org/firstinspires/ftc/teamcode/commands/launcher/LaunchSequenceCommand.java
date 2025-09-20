// Ficheiro: commands/launcher/LaunchSequenceCommand.java
package org.firstinspires.ftc.teamcode.commands.launcher;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.intake.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class LaunchSequenceCommand extends SequentialCommandGroup {
    public LaunchSequenceCommand(LauncherSubsystem launcher, IntakeSubsystem intake) {
        addCommands(
                new InstantCommand(launcher::spinUp, launcher),
                new WaitCommand(Constants.Launcher.SPIN_UP_TIME_MS),
                new RunIntakeCommand(intake, RunIntakeCommand.IntakeAction.RUN),
                new WaitCommand(Constants.Launcher.FEED_TIME_MS),
                new RunIntakeCommand(intake, RunIntakeCommand.IntakeAction.STOP),
                new InstantCommand(launcher::stop, launcher)
        );
    }
}