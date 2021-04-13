package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.ClimbSubsystem;
import frc.team832.robot.subsystems.Climber;

public class StopClimbGroup extends SequentialCommandGroup {

    public StopClimbGroup(ClimbSubsystem climber){
        addCommands(
                new InstantCommand(climber::stopClimb),
                new WaitCommand(0.2),
                new InstantCommand(climber::lockClimb)
        );
    }
}
