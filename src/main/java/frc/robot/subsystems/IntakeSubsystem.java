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

    private final SparkMax IntakeMotor;
    private final SparkMax SpinDexeMotor1;
    private final SparkMax SpinDexeMotor2;
    private final SparkMax ConveyorMotor;
    private final TalonFX KrakenMotor;
    private final SparkMax IndexerMotor;
    private final SparkClosedLoopController revClosedLoopController;

    private final double GEAR_RATIO = 80.0; 

    public IntakeSubsystem() {
        IntakeMotor = new SparkMax(GeneralConstants.IntakeMechanismConstants.kIntakeRotateMotorId, MotorType.kBrushless);
        KrakenMotor = new TalonFX(GeneralConstants.IntakeMechanismConstants.kIntakeMotorId, TunerConstants.kCANBus);
        SpinDexeMotor1 = new SparkMax(GeneralConstants.IntakeMechanismConstants.kSpinDexeMotor1Id, MotorType.kBrushless);
        SpinDexeMotor2 = new SparkMax(GeneralConstants.IntakeMechanismConstants.kSpinDexeMotor2Id, MotorType.kBrushless);
        IndexerMotor = new SparkMax(GeneralConstants.IntakeMechanismConstants.kIndexerMotorId, MotorType.kBrushless);
        ConveyorMotor = new SparkMax(GeneralConstants.IntakeMechanismConstants.kConveyorMotorId, MotorType.kBrushless);
        revClosedLoopController = IntakeMotor.getClosedLoopController();

        SparkMaxConfig revConfig = new SparkMaxConfig();        
        
        revConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(false);

        revConfig.closedLoop
            .pid(0.1, 0.0, 0.0) 
            .outputRange(-0.3, 0.3);

        IntakeMotor.configure(revConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void drivetoAngle(double angle) {
        double targetRotations = angle / 360.0;
        double motorRotations = targetRotations * GEAR_RATIO;
        revClosedLoopController.setSetpoint(motorRotations, ControlType.kPosition);
    }
    
    public void runKrakenMotor(double speed) {
        KrakenMotor.set(speed);
    }

    public void EmptyStorage(){
        SpinDexeMotor1.set(-GeneralConstants.IntakeMechanismConstants.kSpinDexeSpeed);
        SpinDexeMotor2.set(GeneralConstants.IntakeMechanismConstants.kSpinDexeSpeed);
        ConveyorMotor.set(-0.8);
        IndexerMotor.set(-GeneralConstants.IntakeMechanismConstants.kIndexSpeed);
    }
    public double getRevMotorPosition() {
        return IntakeMotor.getEncoder().getPosition();
    }

    // IntakeSubsystem.java içine eklenecek
    public double getIntakeAngle() {
        // Motorun attığı turu dişli oranına bölüp 360 ile çarparak mekanizmanın gerçek açısını buluruz.
        return (IntakeMotor.getEncoder().getPosition() / GEAR_RATIO) * 360.0;
    }

    public void stopMotors() {
        IntakeMotor.stopMotor();
        KrakenMotor.stopMotor();
        SpinDexeMotor1.stopMotor();
        SpinDexeMotor2.stopMotor();
        IndexerMotor.stopMotor();
        ConveyorMotor.stopMotor();
    }
}