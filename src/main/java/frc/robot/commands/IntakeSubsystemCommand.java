package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSubsystemCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    
    // Hedefleri doğrudan derece cinsinden yazıyoruz (Okuması çok daha kolay)
    private final double TARGET_ANGLE = 105; 
    private final double TOLERANCE = 5.0; // Hedefe 5 derece yaklaşıldığında Kraken başlasın

    public IntakeSubsystemCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Hedefe 120 dereceye git komutunu ver
        intakeSubsystem.drivetoAngle(TARGET_ANGLE);
    }

    @Override
    public void execute() {
        // Mekanizmanın o anki gerçek açısını çek
        double currentAngle = intakeSubsystem.getIntakeAngle();
        
        // Eğer mevcut açı, 120 hedefine 5 derece kadar yaklaştıysa (115 ile 125 arası)
        if (Math.abs(currentAngle - TARGET_ANGLE) < TOLERANCE) {
            // Intake açıldı, içeri top alma/verme motorunu çalıştır
            intakeSubsystem.runKrakenMotor(0.5);
        } else {
            // Hedefe henüz varılmadı, güvenlik için bekle
            intakeSubsystem.runKrakenMotor(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}