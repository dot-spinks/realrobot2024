package frc.robot.intake_OTB;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class intake_OTBIOReal implements intake_OTBIO{
private final TalonFX intakeMotor = new TalonFX(Constants.canIDConstants.otbIntakeMotor, "canivore"); //creates motor obj
private final TalonFX pivotMotor = new TalonFX(Constants.canIDConstants.otbIntakePivotMotor, "canivore");  //creates motor obj

    private final TalonFX pivotMotor = new TalonFX(canIDConstants.otbIntakePivotMotor, "canivore");
    private final TalonFX intakeMotor = new TalonFX(canIDConstants.otbIntakeMotor, "rio");
    private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
}
