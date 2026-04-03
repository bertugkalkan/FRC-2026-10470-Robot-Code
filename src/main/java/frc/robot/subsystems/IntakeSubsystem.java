package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.TunerConstants;

// REVLib 2025 Importları
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax RevMotor;
    private final TalonFX KrakenMotor;
    
    private final SparkClosedLoopController revClosedLoopController;

    private final double GEAR_RATIO = 1.0; 

    public IntakeSubsystem() {
        RevMotor = new SparkMax(GeneralConstants.IntakeMechanismConstants.kIntakeRotateMotorId, MotorType.kBrushless);
        KrakenMotor = new TalonFX(GeneralConstants.IntakeMechanismConstants.kIntakeMotorId, TunerConstants.kCANBus);

        revClosedLoopController = RevMotor.getClosedLoopController();

        SparkMaxConfig revConfig = new SparkMaxConfig();        
        
        revConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);

        revConfig.closedLoop
            .pid(0.1, 0.2, 0.1) 
            .outputRange(-0.3, 0.3);

        RevMotor.configure(revConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void drivetoAngle(double angle) {
        double targetRotations = angle / 360.0;
        double motorRotations = targetRotations * GEAR_RATIO;
        revClosedLoopController.setSetpoint(motorRotations, ControlType.kPosition);
    }
    
    public void runKrakenMotor(double speed) {
        KrakenMotor.set(speed);
    }

    public double getRevMotorPosition() {
        return RevMotor.getEncoder().getPosition();
    }

    public void stopMotors() {
        RevMotor.stopMotor();
        KrakenMotor.stopMotor();
    }
}