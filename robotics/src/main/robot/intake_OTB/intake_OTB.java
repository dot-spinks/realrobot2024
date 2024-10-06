package frc.robot.intake_OTB;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.otbIntakeConstants;

// 1 intake pivot 1 roller
public class intake_OTB extends SubsystemBase{
private final TalonFX intakeMotor = new TalonFX(Constants.canIDConstants.otbIntakeMotor, "canivore"); //creates motor obj
private final TalonFXConfigurator intakeConfigurator = intakeMotor.getConfigurator();
private final TalonFXConfiguration intakeConfigs =  new TalonFXConfiguration(); //for configs
private VoltageOut intakeMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true); ; //requests/commands
private final StatusSignal<Double> intakeCurrent = intakeMotor.getStatorCurrent();
private final StatusSignal<Double> intakeRPS = intakeMotor.getRotorVelocity();
private final StatusSignal<Double> intakeTemp = intakeMotor.getDeviceTemp(); //stuff for periodic method

private final TalonFX pivotMotor = new TalonFX(Constants.canIDConstants.otbIntakePivotMotor, "rio" /*this may be wrong */);  //creates motor obj
private final TalonFXConfigurator pivotConfigurator = pivotMotor.getConfigurator();
private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration(); //for configs
private MotionMagicVoltage pivotMotorMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);;
private VoltageOut pivotMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true);
private final StatusSignal<Double> pivotCurrent = pivotMotor.getStatorCurrent();
private final StatusSignal<Double> pivotRPS = pivotMotor.getRotorVelocity();
private final StatusSignal<Double> pivotTemp = pivotMotor.getDeviceTemp();
private final StatusSignal<Double> pivotPosition = pivotMotor.getRotorPosition(); //stuff for periodic method

    public intake_OTB(){

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast; //motor coasts when "off"
        intakeMotorOutputConfigs.Inverted = Constants.otbIntakeConstants.intakeInvert; //

        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = Constants.otbIntakeConstants.intakeCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        intakeConfigurator.apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                intakeCurrent,
                intakeRPS,
                intakeTemp
               ); //periodic

        var pivotMotorOuputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOuputConfigs.Inverted = Constants.otbIntakeConstants.pivotInvert;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit = Constants.otbIntakeConstants.pivotCurrentLimit;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        //slot0Configs.kP = 1; 
        //slot0Configs.kI = 1;
        //slot0Configs.kD = 1;
        //slot0Configs.kS = 1; 
        //slot0Configs.kV = 1; 
        //slot0Configs.kA = 1;
        //slot0Configs.kG = 1;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60;
        motionMagicConfigs.MotionMagicAcceleration = 120;
        motionMagicConfigs.MotionMagicJerk = 10000;

        pivotMotor.setPosition(0);

        pivotConfigurator.apply(pivotConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                pivotCurrent,
                pivotPosition,
                pivotRPS,
                pivotTemp
                
               );



        
    }

    public void periodic(){
BaseStatusSignal.refreshAll(intakeCurrent, intakeRPS, intakeTemp, pivotCurrent, pivotRPS, pivotTemp, pivotPosition);
SmartDashboard.putNumber("Intake Current", intakeCurrent.getValue());
SmartDashboard.putNumber("Intake RPS", intakeRPS.getValue());
SmartDashboard.putNumber("Intake Temp", intakeTemp.getValue());
SmartDashboard.putNumber("Pivot Current", pivotCurrent.getValue());
SmartDashboard.putNumber("Rivot RPS", pivotRPS.getValue());
SmartDashboard.putNumber("Pivot Temp", pivotTemp.getValue());
SmartDashboard.putNumber("Pivot Postion", pivotPosition.getValue());

    }

    public void requestIntakeVoltage(double voltage){
        intakeMotor.setControl(intakeMotorVoltageRequest.withOutput(voltage));
    }

    public void requestPivotVoltage(double voltage){
        pivotMotor.setControl(pivotMotorVoltageRequest.withOutput(voltage));
    }

    public void requestPivotSetPoint(double angleDegrees){
        double pivotSetPointRotation = angleDegrees / (360.0 / Constants.otbIntakeConstants.gearRatio);
        pivotMotor.setControl(pivotMotorMotionMagicRequest.withPosition(pivotSetPointRotation));
    }

    public void requestIntake(double angleDegrees, double voltage){
        requestPivotSetPoint(angleDegrees);
        requestIntakeVoltage(voltage);
    }
    }
