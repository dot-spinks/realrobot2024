package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class canIDConstants{
             /* CAN loop */
     public static final String canivore = "canivore";
     public static final String rio = "rio";
 
     public static final int pigeon = 0;
 
     /* Swerve: FL, FR, BL, BR */
     public static final int[] driveMotor = { 1, 2, 3, 4 };
     public static final int[] steerMotor = { 5, 6, 7, 8 };
     public static final int[] CANcoder = { 9, 10, 11, 12 };
 
     /* OTB_Intake */
     public static final int otbIntakePivotMotor = 13;
     public static final int otbIntakeMotor = 14;
     public static final int indexerMotor = 15;
 
     /* Shooter Arm*/
     public static final int leftShooterMotor = 16;
     public static final int rightShooterMotor = 17;
     public static final int leftArmMotor = 18;
     public static final int rightArmMotor = 19;
     public static final int ampRollerMotor = 20;
    }
    
    public static final class indexerConstants {
     /* Motor Inverts */

    public static final InvertedValue indexerInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue ampRollerInvert = InvertedValue.CounterClockwise_Positive; 

    /* Current Limits */

    public static final double indexerCurrentLimit = 50; //NOT REAL
    public static final double ampRollerCurrentLimit = 50; //NOT REAL
}

public static final class otbIntakeConstants {
    /* Motor Inverts */

public static final InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;
public static final InvertedValue intakeInvert = InvertedValue.Clockwise_Positive;

/* Current Limits */

public static final double pivotCurrentLimit = 50; //NOT REAL
public static final double intakeCurrentLimit = 50; //NOT REAL

public static final double gearRatio = 76.1904761905;
}

public static final class shooterConstants {
    
        /* Motor Inverts */
    public static final InvertedValue leftShooterInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightShooterInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue leftArmInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightArmInvert = InvertedValue.CounterClockwise_Positive;

    /* Current Limits */
    public static final double shooterCurrentLimit = 50;
    public static final double armCurrentLimit = 50;

    public static double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI; 
    public static double shooterGearRatio = 0.5;
    public static double armGearRatio = 97.337962963;
}

public class swerveConstants {
     public static final class moduleConstants {
        /* Inverts FL, FR, BL, BR */
        public static final InvertedValue[] driveMotorInverts = {InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive};
        public static final InvertedValue[] steerMotorInverts = {InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive};
        public static final SensorDirectionValue[] CANcoderInverts = {SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive};
        /* CANcoder Offset FL, FR, BL, BR */
        public static final double[] CANcoderOffsets = {0.106201, 0.048828, 0.108643, 0.895264}; 
    
        
        /* Gear Ratios */
        public static final double driveGearRatio = 6.12;
        public static final double steerGearRatio = 150.0 / 7.0;

        /* Max Speeds */
        public static final double maxSpeedMeterPerSecond = Units.feetToMeters(16); 
        public static final double maxAngularVelocity = 2.5; 
        
        /* Current Limits */
        public static final double driveStatorCurrentLimit = 80;
        public static final double steerStatorCurrentLimit = 50;

        /* Ramp Rate */
        public static final double rampRate = 0.02;
        
        /* PID Values */
        public static final double drivekP = 0.0;
        public static final double drivekD = 0.0;
        public static final double drivekS = 0.0;
        public static final double drivekV = 0.0;

        public static final double anglekP = 0.0;
        public static final double anglekD = 0.0;
        public static final double anglekS = 0.0;
        public static final double anglekV = 0.0;

        /* Wheel Circumference */
        public static final double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;
    }

    public static final class kinematicsConstants{
        /* Drivetrain Constants */
        public static final double wheelBase = Units.inchesToMeters(22.75);
        public static final double trackWidth = Units.inchesToMeters(20.75);

        /* Swerve Kinematics */
        public static final Translation2d FL = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d FR = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d BL = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d BR = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

    }
}
}
