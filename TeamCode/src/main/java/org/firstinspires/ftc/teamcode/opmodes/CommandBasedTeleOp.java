// Ficheiro: opmodes/CommandBasedTeleOp.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@TeleOp(name = "DECODE Command-Based TeleOp")
public class CommandBasedTeleOp extends CommandOpMode {
    private RobotContainer robot;

    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);
        robot = new RobotContainer(hardwareMap, driver, operator);
    }
}