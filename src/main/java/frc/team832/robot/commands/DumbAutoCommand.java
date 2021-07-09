package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.SuperStructure;

public class DumbAutoCommand extends SequentialCommandGroup {

    public DumbAutoCommand(DrivetrainSubsystem drivetrain, SuperStructure superStructure) {
        addRequirements(drivetrain, superStructure);
        addCommands(
                new AutoShootCommandGroup(superStructure),
                new InstantCommand(() -> drivetrain.setWheelVolts(-1.0, 1.0),drivetrain),
                new WaitCommand(0.75),
                new InstantCommand(() -> drivetrain.setWheelVolts(0.0, 0.0), drivetrain)
        );
    }
}
