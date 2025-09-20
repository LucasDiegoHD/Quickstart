// Ficheiro: subsystems/LauncherSubsystem.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class LauncherSubsystem extends SubsystemBase {
    private final DcMotorEx launcherMotorLeft;
    private final DcMotorEx launcherMotorRight;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        launcherMotorLeft = hardwareMap.get(DcMotorEx.class, Constants.Launcher.LAUNCHER_MOTOR_LEFT);
        launcherMotorRight = hardwareMap.get(DcMotorEx.class, Constants.Launcher.LAUNCHER_MOTOR_RIGHT);

        launcherMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void spinUp() {
        launcherMotorLeft.setVelocity(Constants.Launcher.LAUNCH_SPEED);
        launcherMotorRight.setVelocity(Constants.Launcher.LAUNCH_SPEED);
    }

    public void stop() {
        launcherMotorLeft.setPower(0);
        launcherMotorRight.setPower(0);
    }
}