package org.firstinspires.ftc.teamcode.commands.deflector;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.DeflectorSubsystem;

public class MoveDeflectorCommand extends InstantCommand {
    public MoveDeflectorCommand(DeflectorSubsystem deflector, double position) {
        super(
                () -> deflector.setPosition(position),
                deflector
        );
    }
}