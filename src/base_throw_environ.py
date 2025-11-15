from pydrake.all import (
    DiagramBuilder,
    Simulator,
    StartMeshcat,
    ConstantVectorSource,
)

from manipulation.station import (
    LoadScenario,
    MakeHardwareStation,
)

from pathlib import Path

# Start meshcat for visualization
meshcat = StartMeshcat()
print("Click the link above to open Meshcat in your browser!")

## Base environment

abs_table_sdf_path = f"{Path.cwd()}/assets/table.sdf"
with open(abs_table_sdf_path, "r") as fin:
    table_sdf = fin.read()

scenario_yaml = f"""directives:
- add_model:
    name: iiwa
    file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
    default_joint_positions:
        iiwa_joint_1: [-1.57]
        iiwa_joint_2: [0.1]
        iiwa_joint_3: [0]
        iiwa_joint_4: [-1.2]
        iiwa_joint_5: [0]
        iiwa_joint_6: [ 1.6]
        iiwa_joint_7: [0]
- add_weld:
    parent: world
    child: iiwa::iiwa_link_0
- add_model:
    name: wsg
    file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
- add_weld:
    parent: iiwa::iiwa_link_7
    child: wsg::body
    X_PC:
        translation: [0, 0, 0.09]
        rotation: !Rpy {{ deg: [90, 0, 90]}}
- add_model:
    name: table
    file: file://{abs_table_sdf_path}
- add_weld:
    parent: world
    child: table::table_link
    X_PC:
        translation: [0.0, 0.0, -0.05]
        rotation: !Rpy {{ deg: [0, 0, -90] }}

model_drivers:
    iiwa: !IiwaDriver
      control_mode: position_only
      hand_model_name: wsg
    wsg: !SchunkWsgDriver {{}}
"""

scenario = LoadScenario(data=scenario_yaml)
station = MakeHardwareStation(scenario, meshcat=meshcat)
builder = DiagramBuilder()
builder.AddSystem(station)

# Add constant commands for the iiwa (7 joints)
iiwa_position_command = builder.AddSystem(ConstantVectorSource([-1.57, 0.1, 0, -1.2, 0, 1.6, 0]))
builder.Connect(iiwa_position_command.get_output_port(), 
                station.GetInputPort("iiwa.position"))

# Add constant command for the WSG gripper
wsg_position_command = builder.AddSystem(ConstantVectorSource([0.1]))  # gripper opening width
builder.Connect(wsg_position_command.get_output_port(),
                station.GetInputPort("wsg.position"))

diagram = builder.Build()

simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(0.1)

input("Press enter to exit... ")
