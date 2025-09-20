// Ficheiro: robot/RobotContainer.java
package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.hardware.HardwareMap;

// --- IMPORTAÇÕES CORRETAS DO PEDRO PATHING 2.0 ---
import com.pedropathing.localization.PoseTracker; // Pose está em 'localization'
import com.pedropathing.geometry.BezierLine;     // Geometria de caminho está em 'path'
import com.pedropathing.geometry.BezierPoint;          // Geometria de caminho está em 'path'
import com.pedropathing.paths.PathChain;     // O objeto final do caminho está em 'paths'

// --- IMPORTAÇÕES CORRETAS DOS NOSSOS COMANDOS E SUBSISTEMAS ---
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.commands.drivetrain.*;
import org.firstinspires.ftc.teamcode.commands.intake.*;
import org.firstinspires.ftc.teamcode.commands.launcher.*;
import org.firstinspires.ftc.teamcode.commands.deflector.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class RobotContainer {
    // Subsistemas
    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final LauncherSubsystem launcher;
    private final DeflectorSubsystem deflector;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator) {
        // Inicializa todos os subsistemas
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        deflector = new DeflectorSubsystem(hardwareMap);

        // Define o comando padrão para a transmissão (controlo do piloto)
        if (driver!= null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));
        }

        // Configura as ligações dos botões do gamepad do operador
        if (operator!= null) {
            configureButtonBindings(operator);
        }
    }

    private void configureButtonBindings(GamepadEx operator) {
        new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new LaunchSequenceCommand(launcher, intake));
        new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new RunIntakeCommand(intake, RunIntakeCommand.IntakeAction.RUN))
                .whenReleased(new RunIntakeCommand(intake, RunIntakeCommand.IntakeAction.STOP));
        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new MoveDeflectorCommand(deflector, Constants.Deflector.AIM_HIGH_POS));
        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new MoveDeflectorCommand(deflector, Constants.Deflector.AIM_LOW_POS));
    }

    // Método que constrói e retorna a sequência de comandos do autônomo
    public Command getAutonomousCommand() {
        drivetrain.getFollower().setStartingPose(Constants.FieldPositions.START_POSE);

        PathChain pathToLaunch = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(new Point(Constants.FieldPositions.START_POSE), new Point(Constants.FieldPositions.LAUNCH_POSE)))
                .build();

        PathChain pathToPark = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(new Point(Constants.FieldPositions.LAUNCH_POSE), new Point(Constants.FieldPositions.PARK_POSE)))
                .build();

        // SequentialCommandGroup executa uma lista de comandos, um após o outro.
        return new SequentialCommandGroup(
                new FollowPathCommand(drivetrain, pathToLaunch),
                new MoveDeflectorCommand(deflector, Constants.Deflector.AIM_HIGH_POS),
                new LaunchSequenceCommand(launcher, intake),
                new FollowPathCommand(drivetrain, pathToPark)
        );
    }
}