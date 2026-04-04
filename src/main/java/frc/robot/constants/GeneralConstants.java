package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;


public class GeneralConstants {
        // 2 YENİ TALON FX
    public static final class IntakeMechanismConstants {
        public static final int kIntakeMotorId = 3;
        public static final int kIntakeRotateMotorId = 4;
        public static final int kSpinDexeMotor1Id = 5;
        public static final int kSpinDexeMotor2Id = 6;
        public static final int kIndexerMotorId = 2;
        public static final int kConveyorMotorId = 1;

        // Talon FX Ortak Konfigürasyonu
        public static final TalonFXConfiguration CustomTalonConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(40))
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake));

        // Hız Değerleri
        // public static final double kTalonSpeed = 0.05;
        public static final double kIntakeSpeed = 0.3;
        public static final double kSpinDexeSpeed = -0.3;
        public static final double kIndexSpeed = 0.7;
        public static final double kTalonSpeed = 0.4;   
}
}
