import pydrake.visualization as mut

import copy
import unittest

from pydrake.lcm import (
    DrakeLcm,
)
from pydrake.systems.lcm import (
    LcmBuses,
)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
)
from pydrake.systems.framework import (
    DiagramBuilder,
)


class TestConfig(unittest.TestCase):

    def test_apply_visualization_config(self):
        """Exercises VisualizationConfig and ApplyVisualizationConfig.
        """
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        plant.Finalize()
        lcm_buses = LcmBuses()
        lcm_buses.Add("default", DrakeLcm())

        config = mut.VisualizationConfig(publish_period=0.25)
        self.assertIn("publish_period", repr(config))
        copy.copy(config)
        mut.ApplyVisualizationConfig(
            config=config,
            plant=plant,
            scene_graph=scene_graph,
            lcm_buses=lcm_buses,
            builder=builder)

    def test_add_default_visualization(self):
        """Exercises AddDefaultVisualization.
        """
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        plant.Finalize()
        config = mut.AddDefaultVisualization(builder=builder)
