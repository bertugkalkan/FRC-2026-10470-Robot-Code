package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants; // TunerConstants'a erişim için

// REVLib 2025 Importları
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class CustomMechanismSubsystem extends SubsystemBase {

    private final SparkMax topRevMotor;

    public CustomMechanismSubsystem() {
        topRevMotor = new SparkMax(TunerConstants.CustomMechanismConstants.kTopSparkId, MotorType.kBrushless);
        
        // Ayar objesini oluşturuyoruz
        SparkMaxConfig revConfig = new SparkMaxConfig();
        revConfig
                .idleMode(IdleMode.kBrake) // Fren modu
                .smartCurrentLimit(40) // 40 Amper sınır
                .inverted(false);

        // Ayarları motora yakıyoruz (Factory Reset ve BurnFlash dahil)
        topRevMotor.configure(revConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Motorları sürmek için metodun
    public void runMotors(double talonSpeed, double sparkSpeed) {
        topRevMotor.set(sparkSpeed);
    }

    public void stopMotors() {
        topRevMotor.stopMotor();
    }
}