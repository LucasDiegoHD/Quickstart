// Ficheiro: subsystems/IntakeSubsystem.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.Intake.Hardware.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() { intakeMotor.setPower(1.0); }
    public void reverse() { intakeMotor.setPower(-1.0); }
    public void stop() { intakeMotor.setPower(0); }
}