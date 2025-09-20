// Ficheiro: subsystems/DeflectorSubsystem.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DeflectorSubsystem extends SubsystemBase {
    private final Servo deflectorServo;

    public DeflectorSubsystem(HardwareMap hardwareMap) {
        deflectorServo = hardwareMap.get(Servo.class, Constants.Deflector.DEFLECTOR_SERVO);
    }

    public void setPosition(double position) {
        deflectorServo.setPosition(position);
    }
}