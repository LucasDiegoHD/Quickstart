// Ficheiro: opmodes/CommandBasedAuto.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@Autonomous(name = "DECODE Command-Based Auto")
public class CommandBasedAuto extends CommandOpMode {
    private RobotContainer robot;
    private Command autonomousCommand;

    @Override
    public void initialize() {
        robot = new RobotContainer(hardwareMap, null, null);
        autonomousCommand = robot.getAutonomousCommand();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        autonomousCommand.schedule();

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}