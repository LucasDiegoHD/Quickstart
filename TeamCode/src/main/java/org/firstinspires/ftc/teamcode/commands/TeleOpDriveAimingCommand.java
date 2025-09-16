package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TeleOpDriveAimingCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;

    private final PIDController angleController;

    private final double targetX;
    private final double targetY;

    private static final double ANGLE_KP = 1.0;
    private static final double ANGLE_KI = 0.0;
    private static final double ANGLE_KD = 0.0;

    public TeleOpDriveAimingCommand(DrivetrainSubsystem drivetrain,
                                        GamepadEx driverGamepad,
                                        double targetX,
                                        double targetY) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        this.targetX = targetX;
        this.targetY = targetY;

        this.angleController = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
        angleController.reset();
    }

    @Override
    public void execute() {
        double forward = -driverGamepad.getLeftY();
        double strafe = driverGamepad.getLeftX();

        double robotX = drivetrain.getFollower().getPose().getX();
        double robotY = drivetrain.getFollower().getPose().getY();
        double robotHeading = drivetrain.getFollower().getHeading();

        double desiredTheta = Math.atan2(targetY - robotY, targetX - robotX);

        double error = desiredTheta - robotHeading;
        error = Math.atan2(Math.sin(error), Math.cos(error));

        double omega = angleController.calculate(0, error);

        double manualTurn = -driverGamepad.getRightX();
        if (Math.abs(manualTurn) > 0.05) {
            omega = manualTurn;
        }

        drivetrain.getFollower().setTeleOpDrive(forward, strafe, omega, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
