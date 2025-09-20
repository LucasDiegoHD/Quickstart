// Ficheiro: commands/intake/RunIntakeCommand.java
package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends InstantCommand {
    public enum IntakeAction { RUN, REVERSE, STOP }

    public RunIntakeCommand(IntakeSubsystem intake, IntakeAction action) {
        super(
                () -> {
                    switch (action) {
                        case RUN: intake.run(); break;
                        case REVERSE: intake.reverse(); break;
                        case STOP: intake.stop(); break;
                    }
                },
                intake
        );
    }
}