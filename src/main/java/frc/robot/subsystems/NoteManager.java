package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoteManager {
    private IntakeSubsystem intake;
    private IndexSubsystem indexer;
    private ShooterSubsystem shooter;
    private ShooterBeamBreak beamBreak;

    public NoteManager() {
        this.intake = new IntakeSubsystem();
        this.indexer = new IndexSubsystem();
        this.shooter = new ShooterSubsystem();
        this.beamBreak = new ShooterBeamBreak();
    }

    public Command setIntakeState() {
        return Commands.run(() -> {
            shooter.setIdleState();
            if(beamBreak.isBroken()) {
                intake.setIdleState();
                indexer.setIdleState();
            } else {
                intake.setIntakeState();
                indexer.setIntakeState();
            }        
        }).handleInterrupt(() -> {
            intake.setIdleState();
            indexer.setIdleState();
        });
    }

    public Command setOuttakeState() {
        return Commands.run(() -> {
            shooter.setIdleState();
            intake.setOuttakeState();
            indexer.setOuttakeState();      
        }).handleInterrupt(() -> {
            intake.setIdleState();
            indexer.setIdleState();
        });
    }

    public Command setIdleState() {
        return Commands.run(() -> {
            shooter.setIdleState();
            intake.setIdleState();
            indexer.setIdleState();
        });
    }
}
