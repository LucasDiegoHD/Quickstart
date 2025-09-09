// Ficheiro: teleop.java
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@TeleOp
public class teleop extends CommandOpMode {

    private RobotContainer robot;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Passa o hardwareMap, a telemetria e os gamepads para o RobotContainer
        robot = new RobotContainer(hardwareMap, telemetry, driverGamepad, operatorGamepad);
    }
}