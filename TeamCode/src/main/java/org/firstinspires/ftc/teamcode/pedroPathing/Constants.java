// Ficheiro: pedroPathing/Constants.java
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Ficheiro de constantes unificado para toda a configuração do robô.
 * Organizado hierarquicamente por subsistema para clareza e manutenção.
 */
public final class Constants {
    private Constants() {} // Impede a instanciação

    /**
     * Constantes para o subsistema Drivetrain.
     */
    public static final class Drivetrain {
        public static final class Hardware {
            public static final String LEFT_FRONT_MOTOR = "leftFront";
            public static final String RIGHT_FRONT_MOTOR = "rightFront";
            public static final String LEFT_REAR_MOTOR = "leftRear";
            public static final String RIGHT_REAR_MOTOR = "rightRear";
            public static final String PINPOINT_LOCALIZER = "pinpoint";
        }

        public static final class PedroPathing {
            public static final FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
                    .mass(13)
                    .forwardZeroPowerAcceleration(-25.138658560079815)
                    .lateralZeroPowerAcceleration(-78.96531426769552)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.05, 0))
                    .headingPIDFCoefficients(new PIDFCoefficients(3, 0.1, 0.2, 0))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.025, 0, 0.0002, 1, 0)
                    );

            public static final MecanumConstants MECANUM_CONSTANTS = new MecanumConstants()
                    .leftFrontMotorName(Hardware.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(Hardware.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(Hardware.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(Hardware.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .xVelocity(82.00584290004647)
                    .yVelocity(65.006779349307)
                    .useBrakeModeInTeleOp(true);

            public static final PinpointConstants LOCALIZER_CONSTANTS = new PinpointConstants()
                    .forwardPodY(-5.5)
                    .strafePodX(7)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName(Hardware.PINPOINT_LOCALIZER)
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                    );

            public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                    0.995, 500, 2.5, 1
            );
        }

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(PedroPathing.FOLLOWER_CONSTANTS, hardwareMap)
                    .mecanumDrivetrain(PedroPathing.MECANUM_CONSTANTS)
                    .pinpointLocalizer(PedroPathing.LOCALIZER_CONSTANTS)
                    .pathConstraints(PedroPathing.PATH_CONSTRAINTS)
                    .build();
        }
    }

    /**
     * Constantes para o subsistema Lift (Elevador).
     */
    public static final class Lift {
        public static final String LIFT_MOTOR = "liftMotor";
        public static final int GROUND_POSITION = 0;
        public static final int SCORE_POSITION = 1200;
    }

    /**
     * Constantes para o subsistema Shooter (Lançador).
     */
    public static final class Shooter {
        public static final String SHOOTER_MOTOR = "shooterMotor";
    }

    /**
     * Constantes para o subsistema Intake (Coletor).
     */
    public static final class Intake {
        public static final String INTAKE_MOTOR = "intakeMotor";
    }

    /**
     * Posições importantes no campo.
     */
    public static final class FieldPositions {
        public static final Pose SCORING_POSITION = new Pose(48, 72, Math.toRadians(90));
    }
}