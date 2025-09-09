package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driver, GamepadEx operator) {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap);

        if (driver!= null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));
        }
        if (operator!= null) {
            configureTeleOpBindings(operator, telemetry);
        }
    }

    private void configureTeleOpBindings(GamepadEx operator, Telemetry telemetry) {
        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(shooter.spinUpCommand());

        new GamepadButton(operator, GamepadKeys.Button.X)
                .whenPressed(shooter.stopCommand());

        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new AlignToAprilTagCommand(drivetrain, vision, telemetry));
    }

    public Command getAutoAvancadoCommand() {
        Pose startPose = new Pose(12, 12, Math.toRadians(90));
        Pose shootingPose = new Pose(12, 60, Math.toRadians(90));
        Pose parkPose = new Pose(12, 84, Math.toRadians(90));

        drivetrain.getFollower().setStartingPose(startPose);

        PathChain driveAndShootPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .addParametricCallback(0.7, () -> shooter.spinUpCommand().schedule())
                .build();

        PathChain parkPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(drivetrain, driveAndShootPath),
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(2000),
                new InstantCommand(() -> { /* Lógica para empurrar a peça */ }, shooter),
                new WaitCommand(250),
                shooter.stopCommand(),
                new FollowPathCommand(drivetrain, parkPath)
        );
    }
    public Command getShootThreeAutoCommand() {
        return new SequentialCommandGroup(
                // 1. Gira o shooter para a velocidade alvo.
                shooter.spinUpCommand(),

                // 2. Espera ATÉ que o shooter atinja a velocidade (com um timeout de segurança de 3s).
                new WaitUntilCommand(() -> shooter.atTargetVelocity(Constants.Shooter.VELOCITY_TOLERANCE))
                        .withTimeout(3000),

                new InstantCommand(() -> { /* intake.feed(); */ }, intake, shooter),
                // 4. Espera um pouco para o shooter recuperar a velocidade.
                new WaitCommand(500), // 500ms de espera

                // --- SEGUNDO TIRO ---
                // 5. Empurra o segundo elemento.
                new InstantCommand(() -> { /* intake.feed(); */ }, intake, shooter),
                new WaitCommand(500),

                // --- TERCEIRO TIRO ---
                // 6. Empurra o terceiro elemento.
                new InstantCommand(() -> { /* intake.feed(); */ }, intake, shooter),
                new WaitCommand(250), // Espera final um pouco menor

                shooter.stopCommand()
        );
    }
}