package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterBeamBreak {
    private DigitalInput beambreak;
    
    public ShooterBeamBreak() {
        this.beambreak = new DigitalInput(BreakConstants.kChannel);
    }

    public boolean isBroken() {
        return !beambreak.get();
    }

    private static final class BreakConstants {
        private static final int kChannel = 0;
    }
}
