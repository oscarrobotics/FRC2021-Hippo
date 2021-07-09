package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.SuperStructure;

public class DumbAutoCommand extends SequentialCommandGroup {

    public DumbAutoCommand(DrivetrainSubsystem drivetrain, SuperStructure superStructure) {
        addRequirements(drivetrain, superStructure);
        addCommands(
//                superStructure.targetingCommand,
//                new WaitCommand(1),
//                superStructure.shootCommand,
//                new WaitCommand(3),
//                new InstantCommand(() -> {COmman})
//                superStructure.idleCommand,
                new InstantCommand(() -> drivetrain.setWheelVolts(-1.5, 1.5),drivetrain),
                new WaitCommand(0.6),
                new InstantCommand(() -> drivetrain.setWheelVolts(0.0, 0.0), drivetrain)
        );
    }
}
