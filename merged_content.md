# https://isaac-sim.github.io/IsaacLab/release/2.3.0/source/deployment/run_docker_example.html

Skip to main content

__Back to top __ `Ctrl`+`K`

[ ![](../../_static/NVIDIA-logo-white.png) ![](../../_static/NVIDIA-logo-black.png) Isaac Lab Documentation ](../../index.html)

  * Version

release/2.3.0 main release/2.1.0 release/2.2.0 v2.2.1 v2.2.0 v2.1.1 v2.1.0 v2.0.2 v2.0.1 v2.0.0 v1.4.1 v1.4.0 v1.3.0 v1.2.0 v1.1.0 v1.0.0



  * [__ GitHub](https://github.com/isaac-sim/IsaacLab "GitHub")
  * [![Isaac Sim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://developer.nvidia.com/isaac-sim "Isaac Sim")
  * [![Stars](https://img.shields.io/github/stars/isaac-sim/IsaacLab?color=fedcba)](https://img.shields.io/github/stars/isaac-sim/IsaacLab?color=fedcba "Stars")



__ `Ctrl`+`K`

Isaac Lab

  * [Isaac Lab Ecosystem](../setup/ecosystem.html)
  * [Local Installation](../setup/installation/index.html) __
    * [Installation using Isaac Sim Pip Package](../setup/installation/pip_installation.html)
    * [Installation using Isaac Sim Pre-built Binaries](../setup/installation/binaries_installation.html)
    * [Installation using Isaac Sim Source Code](../setup/installation/source_installation.html)
    * [Installation using Isaac Lab Pip Packages](../setup/installation/isaaclab_pip_installation.html)
    * [Asset Caching](../setup/installation/asset_caching.html)
  * [Container Deployment](index.html) __
    * [Docker Guide](docker.html)
    * Running an example with Docker
    * [Cluster Guide](cluster.html)
    * [Deploying CloudXR Teleoperation on Kubernetes](cloudxr_teleoperation_cluster.html)
  * [Cloud Deployment](../setup/installation/cloud_installation.html)
  * [Reference Architecture](../refs/reference_architecture/index.html)



Getting Started

  * [Quickstart Guide](../setup/quickstart.html)
  * [Build your Own Project or Task](../overview/own-project/index.html) __
    * [Create new project or task](../overview/own-project/template.html)
    * [Project Structure](../overview/own-project/project_structure.html)
  * [Walkthrough](../setup/walkthrough/index.html) __
    * [Environment Design Background](../setup/walkthrough/concepts_env_design.html)
    * [Classes and Configs](../setup/walkthrough/api_env_design.html)
    * [Environment Design](../setup/walkthrough/technical_env_design.html)
    * [Training the Jetbot: Ground Truth](../setup/walkthrough/training_jetbot_gt.html)
    * [Exploring the RL problem](../setup/walkthrough/training_jetbot_reward_exploration.html)
  * [Tutorials](../tutorials/index.html) __
    * [Creating an empty scene](../tutorials/00_sim/create_empty.html)
    * [Spawning prims into the scene](../tutorials/00_sim/spawn_prims.html)
    * [Deep-dive into AppLauncher](../tutorials/00_sim/launch_app.html)
    * [Adding a New Robot to Isaac Lab](../tutorials/01_assets/add_new_robot.html)
    * [Interacting with a rigid object](../tutorials/01_assets/run_rigid_object.html)
    * [Interacting with an articulation](../tutorials/01_assets/run_articulation.html)
    * [Interacting with a deformable object](../tutorials/01_assets/run_deformable_object.html)
    * [Interacting with a surface gripper](../tutorials/01_assets/run_surface_gripper.html)
    * [Using the Interactive Scene](../tutorials/02_scene/create_scene.html)
    * [Creating a Manager-Based Base Environment](../tutorials/03_envs/create_manager_base_env.html)
    * [Creating a Manager-Based RL Environment](../tutorials/03_envs/create_manager_rl_env.html)
    * [Creating a Direct Workflow RL Environment](../tutorials/03_envs/create_direct_rl_env.html)
    * [Registering an Environment](../tutorials/03_envs/register_rl_env_gym.html)
    * [Training with an RL Agent](../tutorials/03_envs/run_rl_training.html)
    * [Configuring an RL Agent](../tutorials/03_envs/configuring_rl_training.html)
    * [Modifying an existing Direct RL Environment](../tutorials/03_envs/modify_direct_rl_env.html)
    * [Policy Inference in USD Environment](../tutorials/03_envs/policy_inference_in_usd.html)
    * [Adding sensors on a robot](../tutorials/04_sensors/add_sensors_on_robot.html)
    * [Using a task-space controller](../tutorials/05_controllers/run_diff_ik.html)
    * [Using an operational space controller](../tutorials/05_controllers/run_osc.html)
  * [How-to Guides](../how-to/index.html) __
    * [Importing a New Asset](../how-to/import_new_asset.html)
    * [Writing an Asset Configuration](../how-to/write_articulation_cfg.html)
    * [Making a physics prim fixed in the simulation](../how-to/make_fixed_prim.html)
    * [Spawning Multiple Assets](../how-to/multi_asset_spawning.html)
    * [Saving rendered images and 3D re-projection](../how-to/save_camera_output.html)
    * [Find How Many/What Cameras You Should Train With](../how-to/estimate_how_many_cameras_can_run.html)
    * [Configuring Rendering Settings](../how-to/configure_rendering.html)
    * [Creating Visualization Markers](../how-to/draw_markers.html)
    * [Wrapping environments](../how-to/wrap_rl_env.html)
    * [Adding your own learning library](../how-to/add_own_library.html)
    * [Recording Animations of Simulations](../how-to/record_animation.html)
    * [Recording video clips during training](../how-to/record_video.html)
    * [Curriculum Utilities](../how-to/curriculums.html)
    * [Mastering Omniverse for Robotics](../how-to/master_omniverse.html)
    * [Setting up CloudXR Teleoperation](../how-to/cloudxr_teleoperation.html)
    * [Simulation Performance](../how-to/simulation_performance.html)
    * [Optimize Stage Creation](../how-to/optimize_stage_creation.html)
  * [Developer’s Guide](../overview/developer-guide/index.html) __
    * [Setting up Visual Studio Code](../overview/developer-guide/vs_code.html)
    * [Repository organization](../overview/developer-guide/repo_structure.html)
    * [Extension Development](../overview/developer-guide/development.html)



Overview

  * [Core Concepts](../overview/core-concepts/index.html) __
    * [Task Design Workflows](../overview/core-concepts/task_workflows.html)
    * [Actuators](../overview/core-concepts/actuators.html)
    * [Sensors](../overview/core-concepts/sensors/index.html) __
      * [Camera](../overview/core-concepts/sensors/camera.html)
      * [Contact Sensor](../overview/core-concepts/sensors/contact_sensor.html)
      * [Frame Transformer](../overview/core-concepts/sensors/frame_transformer.html)
      * [Inertial Measurement Unit (IMU)](../overview/core-concepts/sensors/imu.html)
      * [Ray Caster](../overview/core-concepts/sensors/ray_caster.html)
    * [Motion Generators](../overview/core-concepts/motion_generators.html)
  * [Available Environments](../overview/environments.html)
  * [Reinforcement Learning](../overview/reinforcement-learning/index.html) __
    * [Reinforcement Learning Scripts](../overview/reinforcement-learning/rl_existing_scripts.html)
    * [Reinforcement Learning Library Comparison](../overview/reinforcement-learning/rl_frameworks.html)
    * [Performance Benchmarks](../overview/reinforcement-learning/performance_benchmarks.html)
    * [Debugging and Training Guide](../overview/reinforcement-learning/training_guide.html)
  * [Imitation Learning](../overview/imitation-learning/index.html) __
    * [Augmented Imitation Learning](../overview/imitation-learning/augmented_imitation.html)
    * [Teleoperation and Imitation Learning with Isaac Lab Mimic](../overview/imitation-learning/teleop_imitation.html)
    * [SkillGen for Automated Demonstration Generation](../overview/imitation-learning/skillgen.html)
  * [Showroom Demos](../overview/showroom.html)
  * [Simple Agents](../overview/simple_agents.html)



Features

  * [Hydra Configuration System](../features/hydra.html)
  * [Multi-GPU and Multi-Node Training](../features/multi_gpu.html)
  * [Population Based Training](../features/population_based_training.html)
  * [Tiled Rendering](../overview/core-concepts/sensors/camera.html)
  * [Ray Job Dispatch and Tuning](../features/ray.html)
  * [Reproducibility and Determinism](../features/reproducibility.html)



Experimental Features

  * [Welcome to the bleeding edge!](../experimental-features/bleeding-edge.html)
  * [Newton Physics Integration](../experimental-features/newton-physics-integration/index.html) __
    * [Installation](../experimental-features/newton-physics-integration/installation.html)
    * [Training Environments](../experimental-features/newton-physics-integration/training-environments.html)
    * [Newton Visualizer](../experimental-features/newton-physics-integration/newton-visualizer.html)
    * [Limitations](../experimental-features/newton-physics-integration/limitations-and-known-bugs.html)
    * [Solver Transitioning](../experimental-features/newton-physics-integration/solver-transitioning.html)
    * [Sim-to-Sim Policy Transfer](../experimental-features/newton-physics-integration/sim-to-sim.html)
    * [Sim-to-Real Policy Transfer](../experimental-features/newton-physics-integration/sim-to-real.html)



Resources

  * [Cloud Deployment](../setup/installation/cloud_installation.html)
  * [Sim2Real Deployment of Policies Trained in Isaac Lab](../policy_deployment/index.html) __
    * [Training & Deploying HOVER Policy](../policy_deployment/00_hover/hover_policy.html)
    * [IO Descriptors 101](../policy_deployment/01_io_descriptors/io_descriptors_101.html)



Migration Guides

  * [From IsaacGymEnvs](../migration/migrating_from_isaacgymenvs.html)
  * [From OmniIsaacGymEnvs](../migration/migrating_from_omniisaacgymenvs.html)
  * [From Orbit](../migration/migrating_from_orbit.html)



Source API

  * [API Reference](../api/index.html) __
    * [isaaclab.app](../api/lab/isaaclab.app.html)
    * [isaaclab.actuators](../api/lab/isaaclab.actuators.html)
    * [isaaclab.assets](../api/lab/isaaclab.assets.html)
    * [isaaclab.controllers](../api/lab/isaaclab.controllers.html)
    * [isaaclab.devices](../api/lab/isaaclab.devices.html)
    * [isaaclab.envs](../api/lab/isaaclab.envs.html)
    * [isaaclab.managers](../api/lab/isaaclab.managers.html)
    * [isaaclab.markers](../api/lab/isaaclab.markers.html)
    * [isaaclab.scene](../api/lab/isaaclab.scene.html)
    * [isaaclab.sensors](../api/lab/isaaclab.sensors.html)
    * [isaaclab.sim](../api/lab/isaaclab.sim.html)
    * [isaaclab.terrains](../api/lab/isaaclab.terrains.html)
    * [isaaclab.utils](../api/lab/isaaclab.utils.html)
    * [isaaclab.envs.mdp](../api/lab/isaaclab.envs.mdp.html)
    * [isaaclab.envs.ui](../api/lab/isaaclab.envs.ui.html)
    * [isaaclab.sensors.patterns](../api/lab/isaaclab.sensors.patterns.html)
    * [isaaclab.sim.converters](../api/lab/isaaclab.sim.converters.html)
    * [isaaclab.sim.schemas](../api/lab/isaaclab.sim.schemas.html)
    * [isaaclab.sim.spawners](../api/lab/isaaclab.sim.spawners.html)
    * [isaaclab_rl](../api/lab_rl/isaaclab_rl.html)
    * [isaaclab_mimic.datagen](../api/lab_mimic/isaaclab_mimic.datagen.html)
    * [isaaclab_mimic.envs](../api/lab_mimic/isaaclab_mimic.envs.html)
    * [isaaclab_tasks.utils](../api/lab_tasks/isaaclab_tasks.utils.html)



References

  * [Additional Resources](../refs/additional_resources.html)
  * [Contribution Guidelines](../refs/contributing.html)
  * [Tricks and Troubleshooting](../refs/troubleshooting.html)
  * [Migration Guide (Isaac Sim)](../refs/migration.html)
  * [Known Issues](../refs/issues.html)
  * [Release Notes](../refs/release_notes.html)
  * [Extensions Changelog](../refs/changelog.html)
  * [License](../refs/license.html)
  * [Bibliography](../refs/bibliography.html)



Project Links

  * [GitHub](https://github.com/isaac-sim/IsaacLab)
  * [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
  * [NVIDIA PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/index.html)



__

  * [ __ Repository ](https://github.com/isaac-sim/IsaacLab "Source repository")
  * [ __ Suggest edit ](https://github.com/isaac-sim/IsaacLab/edit/main/docs/source/deployment/run_docker_example.rst "Suggest edit")
  * [ __ Open issue ](https://github.com/isaac-sim/IsaacLab/issues/new?title=Issue%20on%20page%20%2Fsource/deployment/run_docker_example.html&body=Your%20issue%20content%20here. "Open an issue")



__

  * [ __ .rst ](../../_sources/source/deployment/run_docker_example.rst "Download source file")
  * __ .pdf



__ ______ __

# Running an example with Docker

##  Contents 

  * Building the Container
  * The Code
  * The Code Explained
  * Executing the Script



# Running an example with Docker#

From the root of the Isaac Lab repository, the `docker` directory contains all the Docker relevant files. These include the three files (**Dockerfile** , **docker-compose.yaml** , **.env**) which are used by Docker, and an additional script that we use to interface with them, **container.py**.

In this tutorial, we will learn how to use the Isaac Lab Docker container for development. For a detailed description of the Docker setup, including installation and obtaining access to an Isaac Sim image, please reference the [Docker Guide](docker.html#deployment-docker). For a description of Docker in general, please refer to [their official documentation](https://docs.docker.com/get-started/overview/).

## Building the Container#

To build the Isaac Lab container from the root of the Isaac Lab repository, we will run the following:
    
    
    python docker/container.py start
    

The terminal will first pull the base IsaacSim image, build the Isaac Lab image’s additional layers on top of it, and run the Isaac Lab container. This should take several minutes for the first build but will be shorter in subsequent runs as Docker’s caching prevents repeated work. If we run the command `docker container ls` on the terminal, the output will list the containers that are running on the system. If everything has been set up correctly, a container with the `NAME` **isaac-lab-base** should appear, similar to below:
    
    
    CONTAINER ID   IMAGE               COMMAND   CREATED           STATUS         PORTS     NAMES
    483d1d5e2def   isaac-lab-base      "bash"    30 seconds ago   Up 30 seconds             isaac-lab-base
    

Once the container is up and running, we can enter it from our terminal.
    
    
    python docker/container.py enter
    

On entering the Isaac Lab container, we are in the terminal as the superuser, `root`. This environment contains a copy of the Isaac Lab repository, but also has access to the directories and libraries of Isaac Sim. We can run experiments from this environment using a few convenient aliases that have been put into the `root` **.bashrc**. For instance, we have made the **isaaclab.sh** script usable from anywhere by typing its alias `isaaclab`.

Additionally in the container, we have [bind mounted](https://docs.docker.com/storage/bind-mounts/) the `IsaacLab/source` directory from the host machine. This means that if we modify files under this directory from an editor on the host machine, the changes are reflected immediately within the container without requiring us to rebuild the Docker image.

We will now run a sample script from within the container to demonstrate how to extract artifacts from the Isaac Lab Docker container.

## The Code#

The tutorial corresponds to the `log_time.py` script in the `IsaacLab/scripts/tutorials/00_sim` directory.

Code for log_time.py
    
    
     1# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
     2# All rights reserved.
     3#
     4# SPDX-License-Identifier: BSD-3-Clause
     5
     6"""
     7This script demonstrates how to generate log outputs while the simulation plays.
     8It accompanies the tutorial on docker usage.
     9
    10.. code-block:: bash
    11
    12    # Usage
    13    ./isaaclab.sh -p scripts/tutorials/00_sim/log_time.py
    14
    15"""
    16
    17"""Launch Isaac Sim Simulator first."""
    18
    19
    20import argparse
    21import os
    22
    23from isaaclab.app import AppLauncher
    24
    25# create argparser
    26parser = argparse.ArgumentParser(description="Tutorial on creating logs from within the docker container.")
    27# append AppLauncher cli args
    28AppLauncher.add_app_launcher_args(parser)
    29# parse the arguments
    30args_cli = parser.parse_args()
    31# launch omniverse app
    32app_launcher = AppLauncher(args_cli)
    33simulation_app = app_launcher.app
    34
    35"""Rest everything follows."""
    36
    37from isaaclab.sim import SimulationCfg, SimulationContext
    38
    39
    40def main():
    41    """Main function."""
    42    # Specify that the logs must be in logs/docker_tutorial
    43    log_dir_path = os.path.join("logs")
    44    if not os.path.isdir(log_dir_path):
    45        os.mkdir(log_dir_path)
    46    # In the container, the absolute path will be
    47    # /workspace/isaaclab/logs/docker_tutorial, because
    48    # all python execution is done through /workspace/isaaclab/isaaclab.sh
    49    # and the calling process' path will be /workspace/isaaclab
    50    log_dir_path = os.path.abspath(os.path.join(log_dir_path, "docker_tutorial"))
    51    if not os.path.isdir(log_dir_path):
    52        os.mkdir(log_dir_path)
    53    print(f"[INFO] Logging experiment to directory: {log_dir_path}")
    54
    55    # Initialize the simulation context
    56    sim_cfg = SimulationCfg(dt=0.01)
    57    sim = SimulationContext(sim_cfg)
    58    # Set main camera
    59    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])
    60
    61    # Play the simulator
    62    sim.reset()
    63    # Now we are ready!
    64    print("[INFO]: Setup complete...")
    65
    66    # Prepare to count sim_time
    67    sim_dt = sim.get_physics_dt()
    68    sim_time = 0.0
    69
    70    # Open logging file
    71    with open(os.path.join(log_dir_path, "log.txt"), "w") as log_file:
    72        # Simulate physics
    73        while simulation_app.is_running():
    74            log_file.write(f"{sim_time}" + "\n")
    75            # perform step
    76            sim.step()
    77            sim_time += sim_dt
    78
    79
    80if __name__ == "__main__":
    81    # run the main function
    82    main()
    83    # close sim app
    84    simulation_app.close()
    

## The Code Explained#

The Isaac Lab Docker container has several [volumes](https://docs.docker.com/storage/volumes/) to facilitate persistent storage between the host computer and the container. One such volume is the `/workspace/isaaclab/logs` directory. The `log_time.py` script designates this directory as the location to which a `log.txt` should be written:
    
    
        # Specify that the logs must be in logs/docker_tutorial
        log_dir_path = os.path.join("logs")
        if not os.path.isdir(log_dir_path):
            os.mkdir(log_dir_path)
        # In the container, the absolute path will be
        # /workspace/isaaclab/logs/docker_tutorial, because
        # all python execution is done through /workspace/isaaclab/isaaclab.sh
        # and the calling process' path will be /workspace/isaaclab
        log_dir_path = os.path.abspath(os.path.join(log_dir_path, "docker_tutorial"))
        if not os.path.isdir(log_dir_path):
            os.mkdir(log_dir_path)
        print(f"[INFO] Logging experiment to directory: {log_dir_path}")
    

As the comments note, [`os.path.abspath()`](https://docs.python.org/3/library/os.path.html#os.path.abspath "\(in Python v3.13\)") will prepend `/workspace/isaaclab` because in the Docker container all python execution is done through `/workspace/isaaclab/isaaclab.sh`. The output will be a file, `log.txt`, with the `sim_time` written on a newline at every simulation step:
    
    
        # Prepare to count sim_time
        sim_dt = sim.get_physics_dt()
        sim_time = 0.0
    
        # Open logging file
        with open(os.path.join(log_dir_path, "log.txt"), "w") as log_file:
            # Simulate physics
            while simulation_app.is_running():
                log_file.write(f"{sim_time}" + "\n")
                # perform step
                sim.step()
                sim_time += sim_dt
    

## Executing the Script#

We will execute the script to produce a log, adding a `--headless` flag to our execution to prevent a GUI:
    
    
    isaaclab -p scripts/tutorials/00_sim/log_time.py --headless
    

Now `log.txt` will have been produced at `/workspace/isaaclab/logs/docker_tutorial`. If we exit the container by typing `exit`, we will return to `IsaacLab/docker` in our host terminal environment. We can then enter the following command to retrieve our logs from the Docker container and put them on our host machine:
    
    
    ./container.py copy
    

We will see a terminal readout reporting the artifacts we have retrieved from the container. If we navigate to `/isaaclab/docker/artifacts/logs/docker_tutorial`, we will see a copy of the `log.txt` file which was produced by the script above.

Each of the directories under `artifacts` corresponds to Docker [volumes](https://docs.docker.com/storage/volumes/) mapped to directories within the container and the `container.py copy` command copies them from those [volumes](https://docs.docker.com/storage/volumes/) to these directories.

We could return to the Isaac Lab Docker terminal environment by running `container.py enter` again, but we have retrieved our logs and wish to go inspect them. We can stop the Isaac Lab Docker container with the following command:
    
    
    ./container.py stop
    

This will bring down the Docker Isaac Lab container. The image will persist and remain available for further use, as will the contents of any [volumes](https://docs.docker.com/storage/volumes/). If we wish to free up the disk space taken by the image, (~20.1GB), and do not mind repeating the build process when we next run `./container.py start`, we may enter the following command to delete the **isaac-lab-base** image:
    
    
    docker image rm isaac-lab-base
    

A subsequent run of `docker image ls` will show that the image tagged **isaac-lab-base** is now gone. We can repeat the process for the underlying NVIDIA container if we wish to free up more space. If a more powerful method of freeing resources from Docker is desired, please consult the documentation for the [docker prune](https://docs.docker.com/config/pruning/) commands.

[ __ previous Docker Guide ](docker.html "previous page") [ next Cluster Guide __](cluster.html "next page")

__Contents

  * Building the Container
  * The Code
  * The Code Explained
  * Executing the Script



By The Isaac Lab Project Developers. 

© Copyright 2022-2025, The Isaac Lab Project Developers..   


Last updated on Sep 30, 2025.   



---

# https://isaac-sim.github.io/IsaacLab/release/2.3.0/source/deployment/cluster.html

Skip to main content

__Back to top __ `Ctrl`+`K`

[ ![](../../_static/NVIDIA-logo-white.png) ![](../../_static/NVIDIA-logo-black.png) Isaac Lab Documentation ](../../index.html)

  * Version

release/2.3.0 main release/2.1.0 release/2.2.0 v2.2.1 v2.2.0 v2.1.1 v2.1.0 v2.0.2 v2.0.1 v2.0.0 v1.4.1 v1.4.0 v1.3.0 v1.2.0 v1.1.0 v1.0.0



  * [__ GitHub](https://github.com/isaac-sim/IsaacLab "GitHub")
  * [![Isaac Sim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://developer.nvidia.com/isaac-sim "Isaac Sim")
  * [![Stars](https://img.shields.io/github/stars/isaac-sim/IsaacLab?color=fedcba)](https://img.shields.io/github/stars/isaac-sim/IsaacLab?color=fedcba "Stars")



__ `Ctrl`+`K`

Isaac Lab

  * [Isaac Lab Ecosystem](../setup/ecosystem.html)
  * [Local Installation](../setup/installation/index.html) __
    * [Installation using Isaac Sim Pip Package](../setup/installation/pip_installation.html)
    * [Installation using Isaac Sim Pre-built Binaries](../setup/installation/binaries_installation.html)
    * [Installation using Isaac Sim Source Code](../setup/installation/source_installation.html)
    * [Installation using Isaac Lab Pip Packages](../setup/installation/isaaclab_pip_installation.html)
    * [Asset Caching](../setup/installation/asset_caching.html)
  * [Container Deployment](index.html) __
    * [Docker Guide](docker.html)
    * [Running an example with Docker](run_docker_example.html)
    * Cluster Guide
    * [Deploying CloudXR Teleoperation on Kubernetes](cloudxr_teleoperation_cluster.html)
  * [Cloud Deployment](../setup/installation/cloud_installation.html)
  * [Reference Architecture](../refs/reference_architecture/index.html)



Getting Started

  * [Quickstart Guide](../setup/quickstart.html)
  * [Build your Own Project or Task](../overview/own-project/index.html) __
    * [Create new project or task](../overview/own-project/template.html)
    * [Project Structure](../overview/own-project/project_structure.html)
  * [Walkthrough](../setup/walkthrough/index.html) __
    * [Environment Design Background](../setup/walkthrough/concepts_env_design.html)
    * [Classes and Configs](../setup/walkthrough/api_env_design.html)
    * [Environment Design](../setup/walkthrough/technical_env_design.html)
    * [Training the Jetbot: Ground Truth](../setup/walkthrough/training_jetbot_gt.html)
    * [Exploring the RL problem](../setup/walkthrough/training_jetbot_reward_exploration.html)
  * [Tutorials](../tutorials/index.html) __
    * [Creating an empty scene](../tutorials/00_sim/create_empty.html)
    * [Spawning prims into the scene](../tutorials/00_sim/spawn_prims.html)
    * [Deep-dive into AppLauncher](../tutorials/00_sim/launch_app.html)
    * [Adding a New Robot to Isaac Lab](../tutorials/01_assets/add_new_robot.html)
    * [Interacting with a rigid object](../tutorials/01_assets/run_rigid_object.html)
    * [Interacting with an articulation](../tutorials/01_assets/run_articulation.html)
    * [Interacting with a deformable object](../tutorials/01_assets/run_deformable_object.html)
    * [Interacting with a surface gripper](../tutorials/01_assets/run_surface_gripper.html)
    * [Using the Interactive Scene](../tutorials/02_scene/create_scene.html)
    * [Creating a Manager-Based Base Environment](../tutorials/03_envs/create_manager_base_env.html)
    * [Creating a Manager-Based RL Environment](../tutorials/03_envs/create_manager_rl_env.html)
    * [Creating a Direct Workflow RL Environment](../tutorials/03_envs/create_direct_rl_env.html)
    * [Registering an Environment](../tutorials/03_envs/register_rl_env_gym.html)
    * [Training with an RL Agent](../tutorials/03_envs/run_rl_training.html)
    * [Configuring an RL Agent](../tutorials/03_envs/configuring_rl_training.html)
    * [Modifying an existing Direct RL Environment](../tutorials/03_envs/modify_direct_rl_env.html)
    * [Policy Inference in USD Environment](../tutorials/03_envs/policy_inference_in_usd.html)
    * [Adding sensors on a robot](../tutorials/04_sensors/add_sensors_on_robot.html)
    * [Using a task-space controller](../tutorials/05_controllers/run_diff_ik.html)
    * [Using an operational space controller](../tutorials/05_controllers/run_osc.html)
  * [How-to Guides](../how-to/index.html) __
    * [Importing a New Asset](../how-to/import_new_asset.html)
    * [Writing an Asset Configuration](../how-to/write_articulation_cfg.html)
    * [Making a physics prim fixed in the simulation](../how-to/make_fixed_prim.html)
    * [Spawning Multiple Assets](../how-to/multi_asset_spawning.html)
    * [Saving rendered images and 3D re-projection](../how-to/save_camera_output.html)
    * [Find How Many/What Cameras You Should Train With](../how-to/estimate_how_many_cameras_can_run.html)
    * [Configuring Rendering Settings](../how-to/configure_rendering.html)
    * [Creating Visualization Markers](../how-to/draw_markers.html)
    * [Wrapping environments](../how-to/wrap_rl_env.html)
    * [Adding your own learning library](../how-to/add_own_library.html)
    * [Recording Animations of Simulations](../how-to/record_animation.html)
    * [Recording video clips during training](../how-to/record_video.html)
    * [Curriculum Utilities](../how-to/curriculums.html)
    * [Mastering Omniverse for Robotics](../how-to/master_omniverse.html)
    * [Setting up CloudXR Teleoperation](../how-to/cloudxr_teleoperation.html)
    * [Simulation Performance](../how-to/simulation_performance.html)
    * [Optimize Stage Creation](../how-to/optimize_stage_creation.html)
  * [Developer’s Guide](../overview/developer-guide/index.html) __
    * [Setting up Visual Studio Code](../overview/developer-guide/vs_code.html)
    * [Repository organization](../overview/developer-guide/repo_structure.html)
    * [Extension Development](../overview/developer-guide/development.html)



Overview

  * [Core Concepts](../overview/core-concepts/index.html) __
    * [Task Design Workflows](../overview/core-concepts/task_workflows.html)
    * [Actuators](../overview/core-concepts/actuators.html)
    * [Sensors](../overview/core-concepts/sensors/index.html) __
      * [Camera](../overview/core-concepts/sensors/camera.html)
      * [Contact Sensor](../overview/core-concepts/sensors/contact_sensor.html)
      * [Frame Transformer](../overview/core-concepts/sensors/frame_transformer.html)
      * [Inertial Measurement Unit (IMU)](../overview/core-concepts/sensors/imu.html)
      * [Ray Caster](../overview/core-concepts/sensors/ray_caster.html)
    * [Motion Generators](../overview/core-concepts/motion_generators.html)
  * [Available Environments](../overview/environments.html)
  * [Reinforcement Learning](../overview/reinforcement-learning/index.html) __
    * [Reinforcement Learning Scripts](../overview/reinforcement-learning/rl_existing_scripts.html)
    * [Reinforcement Learning Library Comparison](../overview/reinforcement-learning/rl_frameworks.html)
    * [Performance Benchmarks](../overview/reinforcement-learning/performance_benchmarks.html)
    * [Debugging and Training Guide](../overview/reinforcement-learning/training_guide.html)
  * [Imitation Learning](../overview/imitation-learning/index.html) __
    * [Augmented Imitation Learning](../overview/imitation-learning/augmented_imitation.html)
    * [Teleoperation and Imitation Learning with Isaac Lab Mimic](../overview/imitation-learning/teleop_imitation.html)
    * [SkillGen for Automated Demonstration Generation](../overview/imitation-learning/skillgen.html)
  * [Showroom Demos](../overview/showroom.html)
  * [Simple Agents](../overview/simple_agents.html)



Features

  * [Hydra Configuration System](../features/hydra.html)
  * [Multi-GPU and Multi-Node Training](../features/multi_gpu.html)
  * [Population Based Training](../features/population_based_training.html)
  * [Tiled Rendering](../overview/core-concepts/sensors/camera.html)
  * [Ray Job Dispatch and Tuning](../features/ray.html)
  * [Reproducibility and Determinism](../features/reproducibility.html)



Experimental Features

  * [Welcome to the bleeding edge!](../experimental-features/bleeding-edge.html)
  * [Newton Physics Integration](../experimental-features/newton-physics-integration/index.html) __
    * [Installation](../experimental-features/newton-physics-integration/installation.html)
    * [Training Environments](../experimental-features/newton-physics-integration/training-environments.html)
    * [Newton Visualizer](../experimental-features/newton-physics-integration/newton-visualizer.html)
    * [Limitations](../experimental-features/newton-physics-integration/limitations-and-known-bugs.html)
    * [Solver Transitioning](../experimental-features/newton-physics-integration/solver-transitioning.html)
    * [Sim-to-Sim Policy Transfer](../experimental-features/newton-physics-integration/sim-to-sim.html)
    * [Sim-to-Real Policy Transfer](../experimental-features/newton-physics-integration/sim-to-real.html)



Resources

  * [Cloud Deployment](../setup/installation/cloud_installation.html)
  * [Sim2Real Deployment of Policies Trained in Isaac Lab](../policy_deployment/index.html) __
    * [Training & Deploying HOVER Policy](../policy_deployment/00_hover/hover_policy.html)
    * [IO Descriptors 101](../policy_deployment/01_io_descriptors/io_descriptors_101.html)



Migration Guides

  * [From IsaacGymEnvs](../migration/migrating_from_isaacgymenvs.html)
  * [From OmniIsaacGymEnvs](../migration/migrating_from_omniisaacgymenvs.html)
  * [From Orbit](../migration/migrating_from_orbit.html)



Source API

  * [API Reference](../api/index.html) __
    * [isaaclab.app](../api/lab/isaaclab.app.html)
    * [isaaclab.actuators](../api/lab/isaaclab.actuators.html)
    * [isaaclab.assets](../api/lab/isaaclab.assets.html)
    * [isaaclab.controllers](../api/lab/isaaclab.controllers.html)
    * [isaaclab.devices](../api/lab/isaaclab.devices.html)
    * [isaaclab.envs](../api/lab/isaaclab.envs.html)
    * [isaaclab.managers](../api/lab/isaaclab.managers.html)
    * [isaaclab.markers](../api/lab/isaaclab.markers.html)
    * [isaaclab.scene](../api/lab/isaaclab.scene.html)
    * [isaaclab.sensors](../api/lab/isaaclab.sensors.html)
    * [isaaclab.sim](../api/lab/isaaclab.sim.html)
    * [isaaclab.terrains](../api/lab/isaaclab.terrains.html)
    * [isaaclab.utils](../api/lab/isaaclab.utils.html)
    * [isaaclab.envs.mdp](../api/lab/isaaclab.envs.mdp.html)
    * [isaaclab.envs.ui](../api/lab/isaaclab.envs.ui.html)
    * [isaaclab.sensors.patterns](../api/lab/isaaclab.sensors.patterns.html)
    * [isaaclab.sim.converters](../api/lab/isaaclab.sim.converters.html)
    * [isaaclab.sim.schemas](../api/lab/isaaclab.sim.schemas.html)
    * [isaaclab.sim.spawners](../api/lab/isaaclab.sim.spawners.html)
    * [isaaclab_rl](../api/lab_rl/isaaclab_rl.html)
    * [isaaclab_mimic.datagen](../api/lab_mimic/isaaclab_mimic.datagen.html)
    * [isaaclab_mimic.envs](../api/lab_mimic/isaaclab_mimic.envs.html)
    * [isaaclab_tasks.utils](../api/lab_tasks/isaaclab_tasks.utils.html)



References

  * [Additional Resources](../refs/additional_resources.html)
  * [Contribution Guidelines](../refs/contributing.html)
  * [Tricks and Troubleshooting](../refs/troubleshooting.html)
  * [Migration Guide (Isaac Sim)](../refs/migration.html)
  * [Known Issues](../refs/issues.html)
  * [Release Notes](../refs/release_notes.html)
  * [Extensions Changelog](../refs/changelog.html)
  * [License](../refs/license.html)
  * [Bibliography](../refs/bibliography.html)



Project Links

  * [GitHub](https://github.com/isaac-sim/IsaacLab)
  * [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
  * [NVIDIA PhysX](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/index.html)



__

  * [ __ Repository ](https://github.com/isaac-sim/IsaacLab "Source repository")
  * [ __ Suggest edit ](https://github.com/isaac-sim/IsaacLab/edit/main/docs/source/deployment/cluster.rst "Suggest edit")
  * [ __ Open issue ](https://github.com/isaac-sim/IsaacLab/issues/new?title=Issue%20on%20page%20%2Fsource/deployment/cluster.html&body=Your%20issue%20content%20here. "Open an issue")



__

  * [ __ .rst ](../../_sources/source/deployment/cluster.rst "Download source file")
  * __ .pdf



__ ______ __

# Cluster Guide

##  Contents 

  * Setup Instructions
    * Configuring the cluster parameters
    * Exporting to singularity image
  * Defining the job parameters
    * For SLURM
    * For PBS
  * Submitting a job



# Cluster Guide#

Clusters are a great way to speed up training and evaluation of learning algorithms. While the Isaac Lab Docker image can be used to run jobs on a cluster, many clusters only support singularity images. This is because [singularity](https://docs.sylabs.io/guides/2.6/user-guide/index.html) is designed for ease-of-use on shared multi-user systems and high performance computing (HPC) environments. It does not require root privileges to run containers and can be used to run user-defined containers.

Singularity is compatible with all Docker images. In this section, we describe how to convert the Isaac Lab Docker image into a singularity image and use it to submit jobs to a cluster.

Attention

Cluster setup varies across different institutions. The following instructions have been tested on the [ETH Zurich Euler](https://scicomp.ethz.ch/wiki/Euler) cluster (which uses the SLURM workload manager), and the IIT Genoa Franklin cluster (which uses PBS workload manager).

The instructions may need to be adapted for other clusters. If you have successfully adapted the instructions for another cluster, please consider contributing to the documentation.

## Setup Instructions#

In order to export the Docker Image to a singularity image, [apptainer](https://apptainer.org/) is required. A detailed overview of the installation procedure for `apptainer` can be found in its [documentation](https://www.apptainer.org/docs/admin/main/installation.html#install-ubuntu-packages). For convenience, we summarize the steps here for a local installation:
    
    
    sudo apt update
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y ppa:apptainer/ppa
    sudo apt update
    sudo apt install -y apptainer
    

For simplicity, we recommend that an SSH connection is set up between the local development machine and the cluster. Such a connection will simplify the file transfer and prevent the user cluster password from being requested multiple times.

Attention

The workflow has been tested with:

  * `apptainer version 1.2.5-1.el7` and `docker version 24.0.7`

  * `apptainer version 1.3.4` and `docker version 27.3.1`




In the case of issues, please try to switch to those versions.

### Configuring the cluster parameters#

First, you need to configure the cluster-specific parameters in `docker/cluster/.env.cluster` file. The following describes the parameters that need to be configured:

Parameter | Description  
---|---  
CLUSTER_JOB_SCHEDULER | The job scheduler/workload manager used by your cluster. Currently, we support ‘SLURM’ and ‘PBS’ workload managers.  
CLUSTER_ISAAC_SIM_CACHE_DIR | The directory on the cluster where the Isaac Sim cache is stored. This directory has to end on `docker-isaac-sim`. It will be copied to the compute node and mounted into the singularity container. This should increase the speed of starting the simulation.  
CLUSTER_ISAACLAB_DIR | The directory on the cluster where the Isaac Lab logs are stored. This directory has to end on `isaaclab`. It will be copied to the compute node and mounted into the singularity container. When a job is submitted, the latest local changes will be copied to the cluster to a new directory in the format `${CLUSTER_ISAACLAB_DIR}_${datetime}` with the date and time of the job submission. This allows to run multiple jobs with different code versions at the same time.  
CLUSTER_LOGIN | The login to the cluster. Typically, this is the user and cluster names, e.g., `your_user@euler.ethz.ch`.  
CLUSTER_SIF_PATH | The path on the cluster where the singularity image will be stored. The image will be copied to the compute node but not uploaded again to the cluster when a job is submitted.  
REMOVE_CODE_COPY_AFTER_JOB | Whether the copied code should be removed after the job is finished or not. The logs from the job will not be deleted as these are saved under the permanent `CLUSTER_ISAACLAB_DIR`. This feature is useful to save disk space on the cluster. If set to `true`, the code copy will be removed.  
CLUSTER_PYTHON_EXECUTABLE | The path within Isaac Lab to the Python executable that should be executed in the submitted job.  
  
When a `job` is submitted, it will also use variables defined in `docker/.env.base`, though these should be correct by default.

### Exporting to singularity image#

Next, we need to export the Docker image to a singularity image and upload it to the cluster. This step is only required once when the first job is submitted or when the Docker image is updated. For instance, due to an upgrade of the Isaac Sim version, or additional requirements for your project.

To export to a singularity image, execute the following command:
    
    
    ./docker/cluster/cluster_interface.sh push [profile]
    

This command will create a singularity image under `docker/exports` directory and upload it to the defined location on the cluster. It requires that you have previously built the image with the `container.py` interface. Be aware that creating the singularity image can take a while. `[profile]` is an optional argument that specifies the container profile to be used. If no profile is specified, the default profile `base` will be used.

Note

By default, the singularity image is created without root access by providing the `--fakeroot` flag to the `apptainer build` command. In case the image creation fails, you can try to create it with root access by removing the flag in `docker/cluster/cluster_interface.sh`.

## Defining the job parameters#

The job parameters need to be defined based on the job scheduler used by your cluster. You only need to update the appropriate script for the scheduler available to you.

  * For SLURM, update the parameters in `docker/cluster/submit_job_slurm.sh`.

  * For PBS, update the parameters in `docker/cluster/submit_job_pbs.sh`.




### For SLURM#

The job parameters are defined inside the `docker/cluster/submit_job_slurm.sh`. A typical SLURM operation requires specifying the number of CPUs and GPUs, the memory, and the time limit. For more information, please check the [SLURM documentation](https://www.slurm.schedmd.com/sbatch.html).

The default configuration is as follows:
    
    
    12#SBATCH --cpus-per-task=8
    13#SBATCH --gpus=rtx_3090:1
    14#SBATCH --time=23:00:00
    15#SBATCH --mem-per-cpu=4048
    16#SBATCH --mail-type=END
    17#SBATCH --mail-user=name@mail
    18#SBATCH --job-name="training-$(date +"%Y-%m-%dT%H:%M")"
    

An essential requirement for the cluster is that the compute node has access to the internet at all times. This is required to load assets from the Nucleus server. For some cluster architectures, extra modules must be loaded to allow internet access.

For instance, on ETH Zurich Euler cluster, the `eth_proxy` module needs to be loaded. This can be done by adding the following line to the `submit_job_slurm.sh` script:
    
    
    3# in the case you need to load specific modules on the cluster, add them here
    4# e.g., `module load eth_proxy`
    

### For PBS#

The job parameters are defined inside the `docker/cluster/submit_job_pbs.sh`. A typical PBS operation requires specifying the number of CPUs and GPUs, and the time limit. For more information, please check the [PBS Official Site](https://openpbs.org/).

The default configuration is as follows:
    
    
    11#PBS -l select=1:ncpus=8:mpiprocs=1:ngpus=1
    12#PBS -l walltime=01:00:00
    13#PBS -j oe
    14#PBS -q gpu
    15#PBS -N isaaclab
    16#PBS -m bea -M "user@mail"
    

## Submitting a job#

To submit a job on the cluster, the following command can be used:
    
    
    ./docker/cluster/cluster_interface.sh job [profile] "argument1" "argument2" ...
    

This command will copy the latest changes in your code to the cluster and submit a job. Please ensure that your Python executable’s output is stored under `isaaclab/logs` as this directory is synced between the compute node and `CLUSTER_ISAACLAB_DIR`.

`[profile]` is an optional argument that specifies which singularity image corresponding to the container profile will be used. If no profile is specified, the default profile `base` will be used. The profile has be defined directlty after the `job` command. All other arguments are passed to the Python executable. If no profile is defined, all arguments are passed to the Python executable.

The training arguments are passed to the Python executable. As an example, the standard ANYmal rough terrain locomotion training can be executed with the following command:
    
    
    ./docker/cluster/cluster_interface.sh job --task Isaac-Velocity-Rough-Anymal-C-v0 --headless --video --enable_cameras
    

The above will, in addition, also render videos of the training progress and store them under `isaaclab/logs` directory.

[ __ previous Running an example with Docker ](run_docker_example.html "previous page") [ next Deploying CloudXR Teleoperation on Kubernetes __](cloudxr_teleoperation_cluster.html "next page")

__Contents

  * Setup Instructions
    * Configuring the cluster parameters
    * Exporting to singularity image
  * Defining the job parameters
    * For SLURM
    * For PBS
  * Submitting a job



By The Isaac Lab Project Developers. 

© Copyright 2022-2025, The Isaac Lab Project Developers..   


Last updated on Sep 30, 2025.   



---

