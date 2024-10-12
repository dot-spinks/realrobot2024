package frc.robot.shooter;
/*
2 shooter - 
2 arm - 
 
*/

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.shooterConstants;

public class shooter extends SubsystemBase {

    private final TalonFX leftShooter = new TalonFX(canIDConstants.leftShooterMotor, "rio");
    private final TalonFX rightShooter = new TalonFX(canIDConstants.rightShooterMotor, "rio");

    private TalonFXConfiguration leftShooterConfigs = new TalonFXConfiguration();
    private TalonFXConfiguration rightShooterConfigs = new TalonFXConfiguration();
    private TalonFXConfigurator leftShooterConfigurator = leftShooter.getConfigurator();
    private TalonFXConfigurator rightShooterConfigurator = leftShooter.getConfigurator();

    private final StatusSignal<Double> leftShooterCurrent = leftShooter.getStatorCurrent();
    private final StatusSignal<Double> rightShooterCurrent = rightShooter.getStatorCurrent();
    private final StatusSignal<Double> leftShooterTemp = leftShooter.getDeviceTemp();
    private final StatusSignal<Double> rightShooterTemp = rightShooter.getDeviceTemp();
    private final StatusSignal<Double> leftShooterSpeedRPS = leftShooter.getRotorVelocity();
    private final StatusSignal<Double> rightShooterSpeedRPS = rightShooter.getRotorVelocity();

    private VoltageOut leftShootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VoltageOut rightShootRequestVoltage = new VoltageOut(0).withEnableFOC(true); //why r there 2
    private VelocityVoltage leftShooterRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightShooterRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);


    private final TalonFX leftArm = new TalonFX(canIDConstants.leftArmMotor, "rio");
    private final TalonFX rightArm = new TalonFX(canIDConstants.rightArmMotor, "rio");

    private TalonFXConfiguration leftArmConfigs = new TalonFXConfiguration();
    private TalonFXConfigurator leftArmConfigurator = leftArm.getConfigurator();

     private final StatusSignal<Double> leftArmCurrent = leftArm.getStatorCurrent();
    private final StatusSignal<Double> rightArmCurrent = rightArm.getStatorCurrent();
    private final StatusSignal<Double> leftArmTemp = leftArm.getDeviceTemp();
    private final StatusSignal<Double> rightArmTemp = rightArm.getDeviceTemp();
    private final StatusSignal<Double> leftArmPos = leftArm.getRotorPosition();
    private final StatusSignal<Double> rightArmPos = rightArm.getRotorPosition();
    private final StatusSignal<Double> leftArmRPS = leftArm.getRotorVelocity();
    private final StatusSignal<Double> rightArmRPS = rightArm.getRotorVelocity();

    private VoltageOut leftArmVoltageRequest = new VoltageOut(0).withEnableFOC(true);
       private MotionMagicVoltage leftArmMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

       private double leftShooterSetpointMPS = 0;
       private double rightShooterSetpointMPS = 0;
       private double leftArmSetpointDegrees = 0;

//cosine erm??
    public shooter (){
        var leftShooterMotorConfigs = leftShooterConfigs.MotorOutput;
        var rightShooterMotorConfigs = rightShooterConfigs.MotorOutput;
    
        leftShooterMotorConfigs.Inverted = Constants.shooterConstants.leftShooterInvert;
        leftShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        rightShooterMotorConfigs.Inverted = shooterConstants.rightShooterInvert;
        rightShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        var leftShooterCurrentConfigs = leftShooterConfigs.CurrentLimits;
        leftShooterCurrentConfigs.StatorCurrentLimit = shooterConstants.shooterCurrentLimit;
        leftShooterCurrentConfigs.StatorCurrentLimitEnable = true;

        var rightShooterCurrentConfigs = rightShooterConfigs.CurrentLimits;
        rightShooterCurrentConfigs.StatorCurrentLimit = shooterConstants.shooterCurrentLimit;
        rightShooterCurrentConfigs.StatorCurrentLimitEnable = true;

        var leftShooterSlot0Configs = leftShooterConfigs.Slot0;
        leftShooterSlot0Configs.kP = 0.0;
        leftShooterSlot0Configs.kI = 0.0;
        leftShooterSlot0Configs.kD = 0.0;
        leftShooterSlot0Configs.kS = 0.0;
        leftShooterSlot0Configs.kV = 0.0;
        leftShooterSlot0Configs.kA = 0.0;

        var rightShooterSlot0Configs = rightShooterConfigs.Slot0;
        rightShooterSlot0Configs.kP = 0.0;
        rightShooterSlot0Configs.kI = 0.0;
        rightShooterSlot0Configs.kD = 0.0;
        rightShooterSlot0Configs.kS = 0.0;
        rightShooterSlot0Configs.kV = 0.0;
        rightShooterSlot0Configs.kA = 0.0;

        var leftArmMotorConfigs = leftArmConfigs.MotorOutput;

        leftArmMotorConfigs.Inverted = shooterConstants.leftArmInvert;
        leftArmMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        var leftArmCurrentConfigs = leftArmConfigs.CurrentLimits;

        leftArmCurrentConfigs.StatorCurrentLimit = shooterConstants.armCurrentLimit;
        leftArmCurrentConfigs.StatorCurrentLimitEnable = true;

        var leftArmSlot0Configs = leftArmConfigs.Slot0;
        leftArmSlot0Configs.kP = 0.0;
        leftArmSlot0Configs.kI = 0.0;
        leftArmSlot0Configs.kD = 0.0;
        leftArmSlot0Configs.kS = 0.0;
        leftArmSlot0Configs.kV = 0.0;
        leftArmSlot0Configs.kA = 0.0;
        leftArmSlot0Configs.kG = 0.0;
        leftArmSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = leftArmConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        motionMagicConfigs.MotionMagicAcceleration = 0.0;
        motionMagicConfigs.MotionMagicJerk = 0.0;

        var feedbackConfigs = leftArmConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        leftShooter.getConfigurator().apply(leftShooterConfigs);
        rightShooter.getConfigurator().apply(rightShooterConfigs);
        leftArm.getConfigurator().apply(leftArmConfigs);
        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
           leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS,
            leftArmCurrent,
            rightArmCurrent,
            leftArmTemp,
            rightArmTemp,
            leftArmPos,
            rightArmPos,
            leftArmRPS,
            rightArmRPS
            );

        leftShooter.optimizeBusUtilization();
        rightShooter.optimizeBusUtilization();

        leftArmConfigurator.apply(leftArmConfigs);
        leftShooterConfigurator.apply(leftShooterConfigs);
        rightShooterConfigurator.apply(rightShooterConfigs);
        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));
    }
    public void requestShooterVoltage(double voltage) {
        leftShooter.setControl(leftShootRequestVoltage.withOutput(voltage));
        rightShooter.setControl(rightShootRequestVoltage.withOutput(voltage));
    }

    public void requestVelocity(double velocity, double ratio){
        leftShooterSetpointMPS = velocity;
        rightShooterSetpointMPS = velocity * ratio;
        leftShooter.setControl(leftShooterRequestVelocity.withVelocity(Conversions.MPStoRPS(leftShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio)));
        rightShooter.setControl(rightShooterRequestVelocity.withVelocity(Conversions.MPStoRPS(rightShooterSetpointMPS, shooterConstants.wheelCircumferenceMeters, shooterConstants.shooterGearRatio)));
    }

    public void requestArmVoltage(double voltage) {
        leftArm.setControl(leftArmVoltageRequest.withOutput(voltage));
    }

    public void requestSetpoint(double angleDegrees) {
        leftArmSetpointDegrees = angleDegrees;
        double leftArmSetpointRotations = Conversions.DegreesToRotations(angleDegrees, shooterConstants.armGearRatio);
        leftArm.setControl(leftArmMotionMagicRequest.withPosition(leftArmSetpointRotations));
    }

    public void zeroShooterVelocity(){
        leftShooterSetpointMPS = 0;
        rightShooterSetpointMPS = 0;
        leftShooter.setControl(leftShooterRequestVelocity.withVelocity(0));
        rightShooter.setControl(rightShooterRequestVelocity.withVelocity(0));
    }
}
