__author__ = "Brian Liao"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = []
__version__ = "0.1"
__maintainer__ = "Brian Liao"
__email__ = "brian.liao@tum.de"
__status__ = "Integration"

"""
Module test script for environment configuration optimizing
"""
import shutil
from commonroad_rl.train_model import *
from commonroad_rl.tests.common.marker import *
from commonroad_rl.tests.common.path import *

resource_path = resource_root("test_gym_commonroad/pickles")
output_path = output_root("test_reward_configs_opt")

@pytest.mark.parametrize(
    ("env_id"),
    [("commonroad-v1")]
)
@slow
@unit_test
@functional
def test_optimize_reward_configs(env_id):

    # Run training
    meta_scenario_path = os.path.join(resource_path, "meta_scenario")
    train_reset_config_path = os.path.join(resource_path, "problem")
    test_reset_config_path = os.path.join(resource_path, "problem")
    visualization_path = os.path.join(output_path, "images")
    log_path = os.path.join(output_path, "reward_optimization_test_log")

    # Delete output path if already exist
    if os.path.exists(log_path):
        shutil.rmtree(log_path)

    num_of_steps = 1000
    algo = "ppo2"

    sampler = "tpe"
    pruner = "median"

    args_str = (
        f"--seed 5 "
        f"--algo {algo} "
        f"--optimize-reward-configs "
        f"--n-trials 10 "
        f"--n-jobs 1 "
        f"--n-timesteps {num_of_steps} "
        f"--eval-freq 500 "
        f"--eval_episodes 3 "
        f"--n_envs 2 "
        f"--env {env_id} "
        f"--log-folder {log_path} "
        f"--sampler {sampler} "
        f"--pruner {pruner}"
        f" --env-kwargs"
        f' reward_type:"hybrid_reward"'
        f' meta_scenario_path:"{meta_scenario_path}"'
        f' train_reset_config_path:"{train_reset_config_path}"'
        f' test_reset_config_path:"{test_reset_config_path}"'
        f' visualization_path:"{visualization_path}"'
    )

    args = run_stable_baselines_argsparser().parse_args(args_str.split(sep=" "))
    run_stable_baselines(args)

    model_version = sorted(os.listdir(os.path.join(log_path, algo)))[-1]
    file_to_check = f"reward_configuration_optimization/" \
                    f"report_{algo}_{env_id}-10-trials-{num_of_steps}-steps-{sampler}-{pruner}.yml"
    file_path_to_check = os.path.join(log_path, algo, model_version, file_to_check)

    assert os.path.isfile(file_path_to_check)
