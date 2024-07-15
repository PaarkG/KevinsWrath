package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterBeamBreak {
    private DigitalInput beambreak;
    
    public ShooterBeamBreak() {
        this.beambreak = new DigitalInput(BreakConstants.kChannel);
    }

    public BooleanSupplier isBroken() {
        return () -> !beambreak.get();
    }

    public BooleanSupplier isntBroken() {
        return () -> beambreak.get();
    }


    private static final class BreakConstants {
        private static final int kChannel = 0;
    }
}
