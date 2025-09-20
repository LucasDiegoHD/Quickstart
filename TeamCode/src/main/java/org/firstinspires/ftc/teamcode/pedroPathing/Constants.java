// Ficheiro: pedroPathing/Constants.java
package org.firstinspires.ftc.teamcode.pedroPathing;

// Importações do Pedro Pathing
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants; // A importação correta
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;

// Importações do SDK da FTC
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    private Constants() {} // Impede a instanciação

    // ##################################################################################
    // #                             SUBSISTEMA DE TRANSMISSÃO                          #
    // ##################################################################################
    public static class Drivetrain {
        public static class Hardware {
            public static final String LEFT_FRONT_MOTOR = "leftFront";
            public static final String LEFT_REAR_MOTOR = "leftRear";
            public static final String RIGHT_FRONT_MOTOR = "rightFront";
            public static final String RIGHT_REAR_MOTOR = "rightRear";
        }

        public static class PedroPathing {
            public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
                    .mass(16.2)
                    .forwardZeroPowerAcceleration(-25.9)
                    .lateralZeroPowerAcceleration(-67.3)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.015))
                    .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01))
                    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015))
                    .centripetalScaling(0.0005);

            public static MecanumConstants MECANUM_CONSTANTS = new MecanumConstants()
                    .leftFrontMotorName(Hardware.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(Hardware.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(Hardware.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(Hardware.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .xVelocity(78.26)
                    .yVelocity(61.49);

            // *** AQUI ESTÁ A CORREÇÃO MESTRE ***
            // Usamos um bloco de inicialização estático para configurar a classe de dados.
            public static final DriveEncoderConstants LOCALIZER_CONSTANTS;
            static {
                LOCALIZER_CONSTANTS = new DriveEncoderConstants(); // Crie o objeto
                // Defina os campos um por um
                LOCALIZER_CONSTANTS.robot_Length = 18;
                LOCALIZER_CONSTANTS.robot_Width = 18;
                LOCALIZER_CONSTANTS.forwardTicksToInches = 1.0; // OBTENHA COM O AFINADOR
                LOCALIZER_CONSTANTS.strafeTicksToInches = 1.0;  // OBTENHA COM O AFINADOR
                LOCALIZER_CONSTANTS.turnTicksToInches = 1.0;    // OBTENHA COM O AFINADOR
            }

            public static PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                    0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1
            );
        }

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(PedroPathing.FOLLOWER_CONSTANTS, hardwareMap)
                    .mecanumDrivetrain(PedroPathing.MECANUM_CONSTANTS)
                    .driveEncoderLocalizer(PedroPathing.LOCALIZER_CONSTANTS)
                    .pathConstraints(PedroPathing.PATH_CONSTRAINTS)
                    .build();
        }
    }

    // ##################################################################################
    // #                             OUTROS SUBSISTEMAS                                 #
    // ##################################################################################
    public static class Intake {
        public static final String INTAKE_MOTOR = "intakeMotor";
    }

    public static class Launcher {
        public static final String LAUNCHER_MOTOR_LEFT = "launcherMotorLeft";
        public static final String LAUNCHER_MOTOR_RIGHT = "launcherMotorRight";
        public static final double LAUNCH_SPEED = 1800;
        public static final long SPIN_UP_TIME_MS = 1000;
        public static final long FEED_TIME_MS = 500;
    }

    public static class Deflector {
        public static final String DEFLECTOR_SERVO = "deflectorServo";
        public static final double AIM_HIGH_POS = 0.8;
        public static final double AIM_LOW_POS = 0.3;
    }

    // ##################################################################################
    // #                               POSIÇÕES DE CAMPO                                #
    // ##################################################################################
    public static class FieldPositions {
        public static final Pose START_POSE = new Pose(12, 12, Math.toRadians(90));
        public static final Pose LAUNCH_POSE = new Pose(48, 72, Math.toRadians(90));
        public static final Pose PARK_POSE = new Pose(60, 12, Math.toRadians(0));
    }
}