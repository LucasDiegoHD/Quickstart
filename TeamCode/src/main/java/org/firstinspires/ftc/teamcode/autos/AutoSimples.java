// Ficheiro: autos/AutoShootThree.java
package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@Autonomous(name = "Auto: Atirar 3")
public class AutoSimples extends CommandOpMode {

    private RobotContainer robot;
    private Command autonomousCommand;

    @Override
    public void initialize() {
        robot = new RobotContainer(hardwareMap, telemetry, null, null);

        // Pega o NOVO comando que criamos para atirar 3
        autonomousCommand = robot.getShootThreeAutoCommand();

        // Agenda o comando para ser executado após o START
        schedule(autonomousCommand);
    }
}