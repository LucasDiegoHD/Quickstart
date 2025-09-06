package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@Autonomous
public class AutoSimples extends CommandOpMode {

    private RobotContainer robot;
    private Command autonomousCommand;

    @Override
    public void initialize() {
        robot = new RobotContainer(hardwareMap, null, null);
        autonomousCommand = robot.getAutoAvancadoCommand();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        schedule(autonomousCommand);

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}