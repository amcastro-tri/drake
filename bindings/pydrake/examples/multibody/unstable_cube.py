from pydrake.all import *

import time
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

@dataclass
class SimulationExample:
    """A little container for setting up different examples."""
    name: str
    xml: str
    use_hydroelastic: bool
    initial_state: np.array
    sim_time: float

def ball_on_table():
    """A sphere is dropped on a table with some initial horizontal velocity."""
    name = "Ball on table"
    xml = """
    <?xml version="1.0"?>
    <mujoco model="robot">
    <worldbody>
        <geom name="table_top" type="box" pos="0.0 0.0 -0.1" size="0.25 0.25 0.1" rgba="0.9 0.8 0.7 1"/>
        <body>
            <joint type="free"/>
            <geom name="cube" type="box" pos="0.0 0.0 0.05" size="0.05 0.1 0.15" rgba="0.0 1.0 0.0 1"/>
        </body>
    </worldbody>
    </mujoco>
    """
    use_hydroelastic = True
    initial_state = np.array([1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
    sim_time = 1
    return SimulationExample(name, xml, use_hydroelastic, initial_state, sim_time)

#time_step = 0.01
#sap_model = "similar" # sap (default), lagged, or similar
#hydro_modulus = 1.0e7
#hc_dissipation = 50.0
#margin = 0 

#mass=1.0
#length = 0.1
#eps = 0  # Box's length perturbation along x.
#Ly=Lz=length
#Lx = length + eps

#Ix = mass / 12.0 * (Ly*Ly + Lz*Lz)
#Iy = mass / 12.0 * (Lx*Lx + Lz*Lz)
#Iz = mass / 12.0 * (Lx*Lx + Ly*Ly)


class ContactReporter(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self.DeclareAbstractInputPort(
            name="contact_results",
            model_value=Value(
                # Input port will take ContactResults from MultibodyPlant
                ContactResults()))
        # Calling `ForcedPublish()` will trigger the callback.
        self.DeclareForcedPublishEvent(self.Publish)
        
    def Publish(self, context):
        print()
        print(f"ContactReporter::Publish() called at time={context.get_time()}")
        contact_results = self.get_input_port().Eval(context)
        
        num_hydroelastic_contacts = contact_results.num_hydroelastic_contacts()
        print(f"num_hydroelastic_contacts() = {num_hydroelastic_contacts}")
        
        for c in range(num_hydroelastic_contacts):
            print()
            print(f"hydroelastic_contact_info({c}): {c}-th hydroelastic contact patch")
            hydroelastic_contact_info = contact_results.hydroelastic_contact_info(c)
            
            spatial_force = hydroelastic_contact_info.F_Ac_W()
            print("F_Ac_W(): spatial force (on body A, at centroid of contact surface, in World frame) = ")
            print(f"{spatial_force}")
                        
            print("contact_surface()")
            contact_surface = hydroelastic_contact_info.contact_surface()
            num_faces = contact_surface.num_faces()
            total_area = contact_surface.total_area()
            centroid = contact_surface.centroid()
            representation = contact_surface.representation()
            print(f"total_area(): area of contact surface in m^2 = {total_area}")
            print(f"num_faces(): number of polygons or triangles = {num_faces}")
            print(f"centroid(): centroid (in World frame) = {centroid}")  
            print(f"contact surface representation = {representation}")
        
        print()

def add_contact_report(builder, plant):   
    contact_reporter = builder.AddSystem(ContactReporter())    
    builder.Connect(plant.get_contact_results_output_port(),
                    contact_reporter.get_input_port(0))
        
    return builder, plant


def add_contact_viz(builder, plant, meshcat):
    contact_viz = ContactVisualizer.AddToBuilder(
        builder, plant, meshcat,
        ContactVisualizerParams(
            publish_period= 0.01,
            newtons_per_meter= 2e1,
            newton_meters_per_meter= 1e-1))

    return builder, plant

def run_simulation_with_contact_report_and_viz(sim_time, time_step=1e-2):
    clear_meshcat()
    
    builder, plant = add_scene(time_step)
    add_viz(builder, plant)
    add_contact_report(builder, plant)
    add_contact_viz(builder, plant)
    
    diagram = builder.Build()
    
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    
    meshcat.StartRecording(frames_per_second=100.0)
    simulator.AdvanceTo(sim_time)
    meshcat.StopRecording()

    # Numerically report contact results at the end of simulation.
    diagram.ForcedPublish(simulator.get_context())

def create_scene(
    xml: str, 
    time_step: float, 
    hydroelastic: bool = False,
    meshcat: Meshcat = None,
):
    """
    Set up a drake system dyagram

    Args:
        xml: mjcf robot description
        time_step: dt for MultibodyPlant
        hydroelastic: whether to use hydroelastic contact
        meshcat: meshcat instance for visualization. Defaults to no visualization.

    Returns:
        The system diagram, the MbP within that diagram, and the logger instance
        used to keep track of time steps
    """
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=time_step)

    parser = Parser(plant)
    parser.AddModelsFromString(xml, "xml")
    #plant.set_discrete_contact_approximation(
    #        DiscreteContactApproximation.kLagged)
    #plant.set_contact_surface_representation(HydroelasticContactRepresentation.kTriangle)
    plant.Finalize()

    if hydroelastic:
        sg_config = SceneGraphConfig()
        sg_config.default_proximity_properties.compliance_type = "compliant"
        sg_config.default_proximity_properties.hydroelastic_modulus = 1.0e7
        sg_config.default_proximity_properties.hunt_crossley_dissipation = 50.0
        sg_config.default_proximity_properties.dynamic_friction = 1.0
        sg_config.default_proximity_properties.static_friction = 1.0
        sg_config.default_proximity_properties.margin = 0.0
        scene_graph.set_config(sg_config)

    if meshcat is not None:
        #add_contact_viz(builder, plant, meshcat)
        #AddDefaultVisualization(builder=builder, meshcat=meshcat)
        ApplyVisualizationConfig(
            config=VisualizationConfig(
                   publish_period = 0.01,
                   publish_contacts = True),
        builder=builder, meshcat=meshcat)
    
    diagram = builder.Build()
    return diagram, plant

def run_simulation(
    example: SimulationExample,
    visualize: bool = False):
    """
    Run a short simulation, and report the time-steps used throughout.

    Args:
        example: container defining the scenario to simulate
        visualize: flag for playing the sim in meshcat. Note that this breaks
                   timestep visualizations

    Returns:
        Timesteps (dt) throughout the simulation.
    """
    xml = example.xml
    use_hydroelastic = example.use_hydroelastic
    initial_state = example.initial_state
    time_step = 10e-3
    sim_time = example.sim_time

    if visualize:
        meshcat = StartMeshcat()
    else:
        meshcat = None

    # We can use a more standard simulation setup and rely on a logger to
    # tell use the time step information. Note that in this case enabling
    # visualization messes with the time step report though. 
    # Configure Drake's built-in error controlled integration
    config = SimulatorConfig()    
    config.target_realtime_rate = 0.0    
    #config.publish_every_time_step = True

    # Set up the system diagram and initial condition    
    diagram, plant = create_scene(
        xml, time_step, use_hydroelastic, meshcat)    
    context = diagram.CreateDefaultContext()    
    plant_context = diagram.GetMutableSubsystemContext(plant, context)    
    plant.SetPositionsAndVelocities(plant_context, initial_state)
    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(config, simulator)
    simulator.Initialize()
    input("Waiting for meshcat... [ENTER] to continue")

    # Simulate
    if meshcat is not None:
        meshcat.StartRecording()

    start_time = time.time()
    simulator.AdvanceTo(sim_time)
    wall_time = time.time() - start_time

    if meshcat is not None:
        meshcat.StopRecording()
        meshcat.PublishRecording()
    print(f"\nWall clock time: {wall_time}\n")

    PrintSimulatorStatistics(simulator)

    # Keep meshcat instance alive 
    return meshcat


if __name__=="__main__":
    example = ball_on_table()

    meshcat = run_simulation(
        example,
        visualize = True,
    )
