// Ficheiro: robot/RobotContainer.java
package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.lift.MoveLiftToPositionCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class RobotContainer {
    private static final double SHOOTER_TARGET_VELOCITY = 2200.0;

    private final DrivetrainSubsystem drivetrain;
    private final LiftSubsystem lift;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator) {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        if (driver!= null) {
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));
        }
        if (operator!= null) {
            configureTeleOpBindings(operator);
        }
    }

    private void configureTeleOpBindings(GamepadEx operator) {
        // --- CONTROLES DO ELEVADOR ---
        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new MoveLiftToPositionCommand(lift, Constants.Lift.SCORE_POSITION));
        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new MoveLiftToPositionCommand(lift, Constants.Lift.GROUND_POSITION));

        // --- CONTROLES DO LANÇADOR ---
        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(new ShootCommand(shooter, ShootCommand.Action.SPIN_UP, SHOOTER_TARGET_VELOCITY));
        new GamepadButton(operator, GamepadKeys.Button.X)
                .whenPressed(new ShootCommand(shooter, ShootCommand.Action.STOP));

        // --- ASSISTÊNCIA AO PILOTO ---
        new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new GoToPoseCommand(drivetrain, Constants.FieldPositions.SCORING_POSITION));
    }

    /**
     * Retorna o comando autônomo OTIMIZADO.
     * Usa callbacks para executar ações em paralelo, economizando tempo.
     */
    public Command getAutoAvancadoCommand() {
        Pose startPose = new Pose(12, 12, Math.toRadians(90));
        Pose shootingPose = new Pose(12, 60, Math.toRadians(90));
        Pose parkPose = new Pose(12, 84, Math.toRadians(90));

        drivetrain.getFollower().setStartingPose(startPose);

        PathChain driveAndShootPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .addParametricCallback(0.7, 
                        () -> new ShootCommand(shooter, ShootCommand.Action.SPIN_UP, SHOOTER_TARGET_VELOCITY).schedule()
                )
                .build();

        PathChain parkPath = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();

        return new SequentialCommandGroup(
                new FollowPathCommand(drivetrain, driveAndShootPath),
                new WaitCommand(500), // Espera para garantir que o tiro foi concluído
                new ShootCommand(shooter, ShootCommand.Action.STOP),
                new FollowPathCommand(drivetrain, parkPath)
        );
    }
}