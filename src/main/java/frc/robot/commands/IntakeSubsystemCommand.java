package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSubsystemCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double TARGET_ANGLE_ROTATIONS = 120.0 / 360.0; // 120 dereceyi tura çevirdik
    private final double TOLERANCE = 0.02; // Hedefe ne kadar yaklaştığımızda "açık" sayalım?

    public IntakeSubsystemCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        double currentPos = intakeSubsystem.getRevMotorPosition();
        
        if (Math.abs(currentPos - TARGET_ANGLE_ROTATIONS) > TOLERANCE) {
            intakeSubsystem.drivetoAngle(120);
        }
    }

    @Override
    public void execute() {
        // REV motoru hedef açıya (120 derece) ulaştı mı?
        double currentPos = intakeSubsystem.getRevMotorPosition();
        
        if (Math.abs(currentPos - TARGET_ANGLE_ROTATIONS) < TOLERANCE) {
            // Eğer hedef açıdaysak Kraken motorunu çalıştır
            intakeSubsystem.runKrakenMotor(0.5);
        } else {
            // Hedef açıya henüz ulaşılmadıysa Kraken'i güvenlik için durdur veya çalıştırma
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