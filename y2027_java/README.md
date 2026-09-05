# Java Breakout Group Experiment

> [!NOTE]
> This is text-heavy, if you prefer a visual explanation, [one is available](https://www.tldraw.com/f/7dV03rAdjXFDT4cIAtWXm?d=v923.1795.956.824.page)

This experiment is Java, and uses a PubSub-esque architecture, (although not exactly PubSub so we don't have race conditions).

Each tick looks like the following:
    - Update data on DataPollers, calling the updateData method, e.g, this is the 'inputs' into the system
        - Update the current flywheel speed
        - Update the current chassis speed
    - Run each producer, which takes in data, and adds Productions to the ProductionManager
        - Productions are a set of keys and values, alongside a priority
        - Productions are atomic, they either entirely apply, or do not apply at all
        - Lower priority productions that conflict with higher priority productions do not apply at all, ex:
            - Production A sets translation, and has a priority of 1
            - Production B sets translation and rotation and has a priority of 0
            - Due to translation conflicting, Production A, with a higher priority, wins out, and production B does not run at all
            - Rotation remains at the default value due to Production B not running
    - Run each executer, which takes produced data, and updates the physical hardware based on the values the ProductionManager resolved

As a real-world example:
    - Update the DataPollers
        - The ShooterSubsystem updates the current flywheel speed
        - The DriveSubsystem updates the current chassis speeds
    - Run the producers
        - The ShooterControlManager produces a desired flywheel speed
        - The TeleopDriveControlManager produces a desired chassis translation and rotation speed
    - Run each executer
        - The ShooterSubsystem accelerates to the desired flywheel speed
        - The DriveSubsystem accelerates to the desired chassis translation and rotation speeds

The advantage this has over PubSub is that due to the strict priority system, there is no race conditions, values are purely based on the priorities of the productions containing them.
