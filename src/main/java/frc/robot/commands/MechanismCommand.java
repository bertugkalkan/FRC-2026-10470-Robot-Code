package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CustomMechanismSubsystem;

public class MechanismCommand extends Command {
    private final CustomMechanismSubsystem mechanismSubsystem;

    // Constructor: Bu komutun hangi Subsystem'i kullanacağını belirtiyoruz.
    public MechanismCommand(CustomMechanismSubsystem subsystem) {
        this.mechanismSubsystem = subsystem;
        addRequirements(subsystem);
    }

    public void intialize() {
        // Başlangıç için yazılacak kodlar....
    }

    public void execute() {
        mechanismSubsystem.runMotors(0.5, 0.7); // Talon ve REV motorların hızları
    }

    public void end(boolean interrupted) {
        mechanismSubsystem.stopMotors();
    }

    public boolean isFinished() {
        return false; // Bu komut sürekli çalışacak, isFinished() true döndürmez
    }
}