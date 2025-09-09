// Ficheiro: commands/AlignToAprilTagCommand.java
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AlignToAprilTagCommand extends CommandBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final Telemetry telemetry;
    private final PIDController turnController;
    private final PIDController distanceController;

    public AlignToAprilTagCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Telemetry telemetry) {
        this.follower = drivetrain.getFollower();
        this.vision = vision;
        this.telemetry = telemetry;
        turnController = new PIDController(Constants.Vision.TURN_KP, Constants.Vision.TURN_KI, Constants.Vision.TURN_KD);
        distanceController = new PIDController(Constants.Vision.DISTANCE_KP, Constants.Vision.DISTANCE_KI, Constants.Vision.DISTANCE_KD);
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        turnController.setSetPoint(0);
        distanceController.setSetPoint(Constants.Vision.TARGET_TY);
        turnController.setTolerance(0.5);
        distanceController.setTolerance(0.5);
    }

    @Override
    public void execute() {
        turnController.setPID(Constants.Vision.TURN_KP, Constants.Vision.TURN_KI, Constants.Vision.TURN_KD);
        distanceController.setPID(Constants.Vision.DISTANCE_KP, Constants.Vision.DISTANCE_KI, Constants.Vision.DISTANCE_KD);
        distanceController.setSetPoint(Constants.Vision.TARGET_TY);

        if (!vision.hasTarget()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            return;
        }

        double turnPower = turnController.calculate(vision.getTargetTx().orElse(0.0));
        double distancePower = distanceController.calculate(vision.getTargetTy().orElse(Constants.Vision.TARGET_TY));

        telemetry.addData("Align TX", "%.2f", vision.getTargetTx().orElse(999.0));
        telemetry.addData("Align TY", "%.2f", vision.getTargetTy().orElse(999.0));
        telemetry.addData("Turn Power", "%.2f", turnPower);
        telemetry.addData("Distance Power", "%.2f", distancePower);

        follower.setTeleOpDrive(-distancePower, 0, -turnPower, true);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetPoint() && distanceController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
    }
}